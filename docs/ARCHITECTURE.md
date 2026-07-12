# Architecture

## State machine

`src/LR_MS2_BaseCode.cpp` implements the whole rail controller as one hierarchical state machine. Three `enum class`es drive `loop()`, each paired with a `changeX()` helper that sets the value and stamps a `millis()`-based start time, and (for two of them) an `xTimer(duration)` helper for elapsed-time checks:

- **`Mode`** — the top-level state: `HOMING → IDLE ⇄ JOGGING`, plus `ERROR` (terminal; there's no coded path back out of it).
- **`HomingPhase`** — sub-states active only while `mode == HOMING`: `FAST_APPROACH → BACK_OFF → SLOW_APPROACH → SET_ZERO → HOMING_FINISHED`. Fast approach toward the limit switch, back off, slow re-approach for an accurate trigger point, zero both position sources, then a brief LED confirmation before handing off to `IDLE`.
- **`ErrorMode`** — sub-states active only while `mode == ERROR`: `HOMING_TIMEOUT`, `POSITION_MISMATCH`, `ENCODER_NOT_DETECTED`, `OVER_TRAVEL`, `UNKNOWN`. All real error causes light the same LED (`ERROR_LED_PIN1`) — there's only one spare LED, so the serial telemetry is what actually disambiguates which error fired.

`loop()` is one large `switch(mode)`, with a nested `switch(homingPhase)` inside `HOMING` and a nested `switch(errorMode)` inside `ERROR`.

Motion itself is entirely open-loop: `FastAccelStepper` drives the stepper, generating step pulses in an ESP32 hardware peripheral (RMT/MCPWM), and `stepper->getCurrentPosition()` — a step count — is the sole authority on carriage position for control purposes. The state machine is **command-based**: it issues a velocity move (`runForward()`/`runBackward()`, for homing/jogging), a positioning move (`moveTo()`), or an immediate halt (`forceStop()`) only when the intent changes; hardware runs the move in the background and the loop polls `getCurrentPosition()`/`isRunning()`. A small command cache (`commandVelocity` / `commandMoveTo` / `commandHalt`) lets the flat `switch(mode)` keep calling every iteration while only re-issuing to the driver on change. This describes the **open-loop** mode (the default). A second, compile-time-selectable **closed-loop** mode feeds the encoder back into motion — see "Closed-loop position control" below.

## Encoder integration (verification, and now closed-loop feedback)

The AS5600 magnetic encoder was originally added purely to *detect* when the open-loop step count and the real physical position disagree (a skipped step, stall, or slipped belt) — a verification layer, not a driver of motion. In open-loop mode that is still its only role. It is now *also* the feedback signal for the closed-loop mode described below; open-loop step-counting remains the default and is unchanged.

`railEncoder` (a global `RailEncoder` instance, `src/Encoder.cpp` / `include/Encoder.h`) wraps the `robtillaart/AS5600` library so the state-machine file never touches `AS5600`/`Wire` directly. `railEncoder.update()` runs every `loop()` iteration but is internally throttled to a 10ms poll interval — see "Control loop timing" below for why. It's zeroed alongside the stepper's own zero point in `HomingPhase::SET_ZERO`, so the two positions are only meaningfully comparable from `HOMING_FINISHED` onward; the mismatch check is skipped entirely during `HOMING`.

The mismatch check (in `loop()`, gated to `IDLE`/`JOGGING`) compares `stepper->getCurrentPosition()/STEPS_PER_MM` against `railEncoder.positionMm()`:
- **`ENCODER_NOT_DETECTED`** fires immediately and always halts into `Mode::ERROR` if the encoder drops off I2C — including a failed `railEncoder.begin()` at boot.
- **`POSITION_MISMATCH`** fires when the two diverge past `POSITION_MISMATCH_TOLERANCE_MM`, but only actually halts the system if `POSITION_MISMATCH_HALT_ENABLED` is `true`. A running `maxAbsPositionDiffMm` and a live `Would Mismatch` flag are always tracked/printed regardless of the halt toggle, so the mismatch condition can be observed without it stopping the rail.

## Closed-loop position control

A second control mode, layered on the verification encoder above, that uses the encoder as the actual feedback signal. It's selected at **compile time** by the `controlMode` global (`ControlMode::OPEN_LOOP` default / `ControlMode::CLOSED_LOOP`) near the top of `src/LR_MS2_BaseCode.cpp` — flip it and reflash; there is no runtime toggle. `controlModeText()` prints the active mode as the telemetry `ctrl=` field (`OL`/`CL`).

**Open-loop is unchanged.** Every prior motion command is preserved verbatim in the `else` of an `if (controlMode == CLOSED_LOOP)`; closed-loop code executes only in the `CLOSED_LOOP` branch. Homing is always open-loop (the encoder isn't zeroed until `SET_ZERO`, and homing is a physical seek to a switch).

**Setpoints.** Closed-loop servos the encoder onto the same targets the app already commands — the jog soft-limit endpoints (`JOG_MIN/MAX_POSITION_MM`) and the IDLE hold position; no new command interface was added. `clSetpointMm` holds the target. The endpoint branches set it in *both* modes so the telemetry is symmetric for open-vs-closed A/B logging, while the hold latch and all motion effects are closed-loop only.

**Control law** (`closedLoopServoTo`). A proportional servo on encoder error, emitted as an adjusted `moveTo` through the existing `commandMoveTo` cache (it never touches `FastAccelStepper` directly):

```
errMm  = setpointMm - railEncoder.positionMm()      // drive the ENCODER to setpoint
if |errMm| <= CL_DEADBAND_MM: hold last target       // no dither
corrMm = clamp(CL_KP * errMm, +/- CL_MAX_CORRECTION_MM)
commandMoveTo(getCurrentPosition() + mmToSteps(corrMm), speedHz)
```

The clamp keeps the re-planned target close to actual, so a long travel is a smooth cruise (the target sits a fixed distance ahead → constant velocity) that then collapses onto the setpoint and decelerates cleanly in; the deadband nulls hold dither. The servo self-throttles to `CL_UPDATE_INTERVAL_MS` (throttle *inside the callee*, matching the `RailEncoder::update()` / `printStatus()` pattern).

**Stop-and-hold** (`closedLoopHold`). On the first entry to a stop (both jog buttons, jog release, or IDLE) it ramps to a natural stop exactly like the open-loop `commandVelocity(0)` — so the stopping feel is identical and the carriage doesn't snap back — and only once actually stopped latches that rest position as the hold setpoint, then servos to reject drift. `HomingPhase::SET_ZERO` resets `clSetpointMm`/`clInHold` so a re-home re-latches.

`CL_KP`, `CL_DEADBAND_MM`, `CL_MAX_CORRECTION_MM`, `CL_UPDATE_INTERVAL_MS`, and `CL_CORRECTION_SPEED_HZ` (near the top of the file) are **live calibration constants**, like the `POSITION_MISMATCH_*` pair — tune against real hardware (hold dither → raise deadband / lower gain; travel ripple → lower gain / raise clamp/interval). Telemetry adds `sp=` (setpoint) and `cerr=` (setpoint − encoder = the live servo error).

**Mismatch check interaction.** `POSITION_MISMATCH` compares step-count vs. encoder; in closed-loop that difference is exactly the quantity the servo nulls (control error), not a fault. It stays observe-only today (`POSITION_MISMATCH_HALT_ENABLED == false`), so there's no conflict — but if that halt is ever armed, gate it to open-loop only. `ENCODER_NOT_DETECTED` stays active in both modes (a dropped encoder is fatal to closed loop).

## Over-travel failsafe (hard position limits)

Two physical end-of-travel switches bound carriage motion: the home switch (`HOMING_PIN`, the near end) and a far-end switch (`FAR_LIMIT_PIN`). Both are wired NC (`HIGH` = pressed, `INPUT_PULLUP`), so a broken or disconnected switch lead reads as *triggered* — fail-safe.

The failsafe is a **mode-independent guard at the top of `loop()`** (right after `railEncoder.update()`, before `switch(mode)`), separate from the jog logic: if `overTravelTriggered()` (either switch) is true in any mode **except `HOMING` and `ERROR`**, it immediately zeroes speed and latches into `Mode::ERROR` with `ErrorMode::OVER_TRAVEL`. Because it sits above `switch(mode)`, the `ERROR` case runs the same iteration, so no further step pulses issue — the halt is immediate. `HOMING` is excluded so homing can still deliberately drive into the home switch; `ERROR` is excluded so `modeStartTime` isn't re-stamped every cycle once already halted.

- `ERROR` is terminal (no coded path out), so an over-travel trip **latches until reset/re-home**. This is deliberate: a runaway open- or closed-loop controller commanding motion into the limit cannot keep fighting it. This guard is a genuine failsafe layer for any future closed-loop demo, not just jog protection — it required no changes to the jog logic and covers any mode added later automatically.
- Outside homing, the **home switch doubles as the near-end limit**. This is safe because `SET_ZERO` backs the carriage `HOME_PULL_OFF_MM` (5mm) off the switch before zeroing and `HOMING_FINISHED` sends it to 20mm, so the switch is released on entry to `IDLE` — no false trigger right after homing.
- The hard-switch failsafe above is the real safety layer and applies in every non-`HOMING`/`ERROR` mode. On top of it, **`Mode::JOGGING` now also enforces soft (position/step-count-based) limits**: jogging stops and holds at `JOG_MIN_POSITION_MM`/`JOG_MAX_POSITION_MM` (0mm–111.3mm, a 5mm buffer off each switch), so a jog reaching the end of travel simply halts instead of driving into a switch and tripping `OVER_TRAVEL`. These bounds are derived from `HOME_PULL_OFF_MM` and `FAR_LIMIT_POSITION_MM` (the far switch measured at 116.3mm from zero); the far end is a measured constant, not homed to. Soft limits are a jog convenience only — they do not replace the hard-switch failsafe.

## Calibration history

During bring-up, jogging tripped `POSITION_MISMATCH` almost instantly regardless of jog speed or tolerance — the diagnostic culprit turned out to be a **direction inversion**: the encoder's reported position and the stepper's were near-equal magnitude but opposite sign (e.g. `+0.6mm` vs `-0.62mm`). Fixed by passing `AS5600_COUNTERCLOCK_WISE` explicitly to `railEncoder.begin(...)` in `setup()` instead of the library's default `AS5600_CLOCK_WISE`. `POSITION_MISMATCH_TOLERANCE_MM` and `POSITION_MISMATCH_HALT_ENABLED` (both declared near the top of `src/LR_MS2_BaseCode.cpp`) are live calibration constants tuned against real hardware during this process, not fixed forever — check their current values before assuming the mismatch halt is armed.

## Control loop timing

Step-pulse generation no longer competes with anything in `loop()`. Since the migration to `FastAccelStepper`, step pulses are produced by an ESP32 hardware peripheral (RMT/MCPWM), so their timing is **independent of `loop()` / `printStatus()` / encoder-read timing** — `loop()` does not call a motion-service function every iteration; it only issues a command when motion intent changes. This is what lets telemetry run at 10–20 Hz without the print-synced motion stutter the software-stepped `AccelStepper` version suffered (the earlier version toggled the STEP pin from `loop()`, so any contiguous CPU work — formatting a telemetry line, a blocking serial write — stretched a step interval).

Two throttles remain, each for its own reason:
- `RailEncoder::update()` is called unconditionally at the top of `loop()` but throttles itself internally to a 10ms poll interval, because the AS5600 read is a blocking I2C transaction (up to `Wire.setTimeOut(5)` = 5ms on a bad read) and there's no value polling it faster. Keep the throttle *inside* the callee, not as ad-hoc `millis()` checks in `loop()`.
- `printStatus()` is throttled to `PRINT_INTERVAL_MS` (a live-tuned placeholder), since `Serial.print` is comparatively slow. `Serial.setTxBufferSize(1024)` in `setup()` (before `Serial.begin`) additionally gives the UART a background TX ring buffer, so `Serial.print` copies and returns instead of blocking and the UART's own ISR drains it. With hardware step generation this is no longer *required* for smooth motion, but it keeps the loop responsive at high print rates.

## Jog input handling

In `Mode::JOGGING`, the "both jog buttons pressed" case is checked **first** in the if-chain and stops the carriage (`commandVelocity(0)`), so pressing both buttons is a deliberate stop rather than a direction. (Previously this branch sat below the single-button checks and was unreachable dead code; it was moved to the top to make it a real safety stop.)
