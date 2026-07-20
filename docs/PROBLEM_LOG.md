# Problem Log

A running record of non-obvious problems found during bring-up and testing — the symptom, the key
evidence, the root cause, and the fix. Newest first. The point is to capture *why* something happened
(especially when the symptom pointed the wrong way), so the same head-scratch isn't repeated.

---

## 2026-07-20 — Test ramp trips the far over-travel switch with no position mismatch

**Symptom.** During the automated acceleration test (`Mode::TESTING`), the rail consistently latched
`err=OVERTRVL` on the far limit switch — but with **no position mismatch** flagged (`mis=0`). It
looked like an impossible combination (hit the hard limit, yet the sensors say nothing is wrong),
which pointed suspicion at the encoder.

**Key evidence (telemetry at the trip).**
```
...,OL,ERROR,-,OVERTRVL,116.64,0.0,0.00,116.59,0.046,2.665,108.30,-8.294,0,1,0,01,001,2000
             pos^                 enc^   dpos^  dmax^  sp^    cerr^   mis^          accel^
```
- `pos` (step count) and `enc` (encoder) **agree to 0.05 mm** → both position sources healthy; `mis=0`
  is correct, and no steps were lost (a lost-step or bad-encoder case would *desync* them and raise
  `mis`). A manual push was also ruled out — that turns the encoder but not the step counter, which
  would likewise desync them.
- `sp=108.30` (far soft limit) but actual position ≈116.6 → `cerr=-8.294`: the carriage was **driven
  ~8 mm past its setpoint** into the far switch (trigger at `FAR_LIMIT_POSITION_MM = 116.3`).
- `pos` **ramped smoothly** to ~116 (confirmed from the pre-error rows) → the motor was genuinely
  *pulsed* there; this was a real commanded overshoot, not a sensor glitch.
- `accel=2000` = `TEST_ACCEL_START` (the first ramp move). Homing was independently confirmed
  repeatable, ruling out a bad/inconsistent home/zero.

**Root cause.** The **warm-up → first-ramp-move handoff started a move while the carriage was still
moving.** Between ramp moves, `SETTLING` leaves the carriage stopped before the next `moveTo`, but the
`WARMUP` exit called `testBeginMove()` the instant the warm-up timer fired — and the warm-up
oscillation is almost always mid-move at that point. Move 0 uses the very low `TEST_ACCEL_START =
2000` steps/s². If the carriage was moving toward the far end (~4000 steps/s = warm-up speed) at the
handoff, the stop distance is `v²/2a = 4000²/(2·2000) ≈ 4000 steps ≈ 160 mm` — far more than the ~8 mm
left to the 108.3 setpoint. `FastAccelStepper` can't decelerate in time, so it **overshoots the target
and drives into the switch** (it overshoots and reverses when a `moveTo` can't be stopped at the set
acceleration). The motor executes the overshoot faithfully, so `pos`/`enc` climb together and `mis`
never trips. The 8 mm soft-limit buffer is no protection against a ~160 mm potential overshoot.

**Why the symptom misled.** The `mis` flag only compares the two sensors, which both live on the
motor/pulley side of the belt — so "no mismatch" says nothing about whether the *commanded motion*
respected the soft limits. Hitting a limit with `mis=0` isn't contradictory; the two describe
different things ("does the motor track its commands" vs. "did the carriage go where it shouldn't").

**Fix.** Make the warm-up → ramp boundary come to a **full stop before starting move 0**, the same
rest guarantee `SETTLING` gives inter-move starts. In `Mode::TESTING` → `TestPhase::WARMUP`
(`src/LR_MS2_BaseCode.cpp`), once `phaseTimer(TEST_WARMUP_DURATION_MS)` elapses, stop driving warm-up
moves, `commandVelocity(0)`, and wait for `!stepper->isRunning()` before `testBeginMove()`. Moves that
start from rest plan a `moveTo` that always lands on target, so they can't overshoot regardless of the
acceleration. (Inter-move ramp starts and the warm-up-disabled path already start from rest.)

**Takeaway.** Never issue a positioning `moveTo` while the carriage still has velocity toward a nearby
limit — at low acceleration the planned stop distance can exceed the remaining travel. Start moves
from rest, or ensure the remaining distance exceeds `v²/2a`.

---

## 2026-07-20 — Serial telemetry cuts out mid-line after ~1–2 minutes

**Symptom.** The serial stream would stop after a minute or two, at no predictable phase/mode.
Restarting the monitor reset the controller (DTR/RTS on port open), so a run couldn't be recovered.

**Key evidence.** The last line before the stop was **truncated mid-field** (e.g.
`...,108.30,6.25` cut off). A host-side software stall would show *complete* CSV lines up to the stop
(the device only ever sends whole `println`ed lines), so a mid-line cut means the **byte stream was
physically severed** — device or USB-link side, not terminal buffering. No panic/backtrace followed.

**Root cause.** A marginal USB cable — intermittently dropping the link (aggravated by the rig's
vibration at speed). Replacing the cable resolved it.

**Fix / mitigations.**
- New USB data cable (direct motherboard port, no hub). Resolved the dropouts.
- `platformio.ini`: `monitor_dtr = 0` / `monitor_rts = 0` so reopening the monitor does **not** reset
  the ESP32 — lets you reconnect mid-run, and turns a stall into a diagnostic (continuous `millis()`
  timestamp on reconnect = device stayed alive / link glitch; timestamp restarts near 0 = device
  reset).

**Takeaway.** Mid-line truncation vs. clean-line stop is the fast discriminator: truncation = physical
link/device cut; clean lines = host-side read stall. Pin DTR/RTS low so diagnosing doesn't reset the
board.
