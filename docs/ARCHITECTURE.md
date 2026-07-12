# Architecture

## State machine

`src/LR_MS2_BaseCode.cpp` implements the whole rail controller as one hierarchical state machine. Three `enum class`es drive `loop()`, each paired with a `changeX()` helper that sets the value and stamps a `millis()`-based start time, and (for two of them) an `xTimer(duration)` helper for elapsed-time checks:

- **`Mode`** — the top-level state: `HOMING → IDLE ⇄ JOGGING`, plus `ERROR` (terminal; there's no coded path back out of it).
- **`HomingPhase`** — sub-states active only while `mode == HOMING`: `FAST_APPROACH → BACK_OFF → SLOW_APPROACH → SET_ZERO → HOMING_FINISHED`. Fast approach toward the limit switch, back off, slow re-approach for an accurate trigger point, zero both position sources, then a brief LED confirmation before handing off to `IDLE`.
- **`ErrorMode`** — sub-states active only while `mode == ERROR`: `HOMING_TIMEOUT`, `POSITION_MISMATCH`, `ENCODER_NOT_DETECTED`, `OVER_TRAVEL`, `UNKNOWN`. All real error causes light the same LED (`ERROR_LED_PIN1`) — there's only one spare LED, so the serial telemetry is what actually disambiguates which error fired.

`loop()` is one large `switch(mode)`, with a nested `switch(homingPhase)` inside `HOMING` and a nested `switch(errorMode)` inside `ERROR`.

Motion itself is entirely open-loop: `AccelStepper` drives the stepper via `runSpeed()` (homing/jogging) or `run()` (the final approach to a `moveTo()` target), and `stepper.currentPosition()` — a step count — is the sole authority on carriage position for control purposes. There is no closed-loop position control; the encoder (below) is a verification layer only, not a feedback input to motion.

## Encoder integration (verification-only, not closed-loop)

This was a deliberate scope decision: the AS5600 magnetic encoder was added purely to *detect* when the open-loop step count and the real physical position disagree (a skipped step, stall, or slipped belt) — not to replace the stepper's step count as the thing driving motion. There is currently no closed-loop control work in progress; if that's ever wanted, it would be a separate, larger change on top of this.

`railEncoder` (a global `RailEncoder` instance, `src/Encoder.cpp` / `include/Encoder.h`) wraps the `robtillaart/AS5600` library so the state-machine file never touches `AS5600`/`Wire` directly. `railEncoder.update()` runs every `loop()` iteration but is internally throttled to a 10ms poll interval — see "Control loop timing" below for why. It's zeroed alongside the stepper's own zero point in `HomingPhase::SET_ZERO`, so the two positions are only meaningfully comparable from `HOMING_FINISHED` onward; the mismatch check is skipped entirely during `HOMING`.

The mismatch check (in `loop()`, gated to `IDLE`/`JOGGING`) compares `stepper.currentPosition()/STEPS_PER_MM` against `railEncoder.positionMm()`:
- **`ENCODER_NOT_DETECTED`** fires immediately and always halts into `Mode::ERROR` if the encoder drops off I2C — including a failed `railEncoder.begin()` at boot.
- **`POSITION_MISMATCH`** fires when the two diverge past `POSITION_MISMATCH_TOLERANCE_MM`, but only actually halts the system if `POSITION_MISMATCH_HALT_ENABLED` is `true`. A running `maxAbsPositionDiffMm` and a live `Would Mismatch` flag are always tracked/printed regardless of the halt toggle, so the mismatch condition can be observed without it stopping the rail.

## Over-travel failsafe (hard position limits)

Two physical end-of-travel switches bound carriage motion: the home switch (`HOMING_PIN`, the near end) and a far-end switch (`FAR_LIMIT_PIN`). Both are wired NC (`HIGH` = pressed, `INPUT_PULLUP`), so a broken or disconnected switch lead reads as *triggered* — fail-safe.

The failsafe is a **mode-independent guard at the top of `loop()`** (right after `railEncoder.update()`, before `switch(mode)`), separate from the jog logic: if `overTravelTriggered()` (either switch) is true in any mode **except `HOMING` and `ERROR`**, it immediately zeroes speed and latches into `Mode::ERROR` with `ErrorMode::OVER_TRAVEL`. Because it sits above `switch(mode)`, the `ERROR` case runs the same iteration, so no further step pulses issue — the halt is immediate. `HOMING` is excluded so homing can still deliberately drive into the home switch; `ERROR` is excluded so `modeStartTime` isn't re-stamped every cycle once already halted.

- `ERROR` is terminal (no coded path out), so an over-travel trip **latches until reset/re-home**. This is deliberate: a runaway open- or closed-loop controller commanding motion into the limit cannot keep fighting it. This guard is a genuine failsafe layer for any future closed-loop demo, not just jog protection — it required no changes to the jog logic and covers any mode added later automatically.
- Outside homing, the **home switch doubles as the near-end limit**. This is safe because `SET_ZERO` backs the carriage `HOME_PULL_OFF_MM` (5mm) off the switch before zeroing and `HOMING_FINISHED` sends it to 20mm, so the switch is released on entry to `IDLE` — no false trigger right after homing.
- The hard-switch failsafe above is the real safety layer and applies in every non-`HOMING`/`ERROR` mode. On top of it, **`Mode::JOGGING` now also enforces soft (position/step-count-based) limits**: jogging stops and holds at `JOG_MIN_POSITION_MM`/`JOG_MAX_POSITION_MM` (0mm–111.3mm, a 5mm buffer off each switch), so a jog reaching the end of travel simply halts instead of driving into a switch and tripping `OVER_TRAVEL`. These bounds are derived from `HOME_PULL_OFF_MM` and `FAR_LIMIT_POSITION_MM` (the far switch measured at 116.3mm from zero); the far end is a measured constant, not homed to. Soft limits are a jog convenience only — they do not replace the hard-switch failsafe.

## Calibration history

During bring-up, jogging tripped `POSITION_MISMATCH` almost instantly regardless of jog speed or tolerance — the diagnostic culprit turned out to be a **direction inversion**: the encoder's reported position and the stepper's were near-equal magnitude but opposite sign (e.g. `+0.6mm` vs `-0.62mm`). Fixed by passing `AS5600_COUNTERCLOCK_WISE` explicitly to `railEncoder.begin(...)` in `setup()` instead of the library's default `AS5600_CLOCK_WISE`. `POSITION_MISMATCH_TOLERANCE_MM` and `POSITION_MISMATCH_HALT_ENABLED` (both declared near the top of `src/LR_MS2_BaseCode.cpp`) are live calibration constants tuned against real hardware during this process, not fixed forever — check their current values before assuming the mismatch halt is armed.

## Control loop timing

`loop()` has to serve two competing constraints every cycle: keep the stepper's step pulses timely (`AccelStepper.runSpeed()`/`run()` are called every iteration and are timing-sensitive — at `JOG_SPEED`, step pulses are only ~2.5ms apart at the values used during bring-up), and read the AS5600 over I2C, which is a blocking transaction that can stall for up to `Wire.setTimeOut(5)` = 5ms on a bad read. To avoid the encoder read starving step timing, `RailEncoder::update()` throttles itself internally to a 10ms poll interval — callers just call it unconditionally at the top of `loop()`, before `switch(mode)`, so it keeps tracking through every mode without needing per-call throttling logic at the call site. `printStatus()` telemetry is separately throttled to `PRINT_INTERVAL_MS = 250ms`, since `Serial.print` is comparatively slow and doesn't need to run every cycle.

## Known pre-existing quirk

In `Mode::JOGGING`, the "both jog buttons pressed" safety branch is unreachable dead code — the single-button `if` above it already catches that case first. This predates the encoder work and hasn't been asked to be fixed.
