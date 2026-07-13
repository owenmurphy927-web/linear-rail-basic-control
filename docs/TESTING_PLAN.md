# Linear Rail Testing Plan

*Owen Murphy — 2026-07-13*

Focused on **acceleration and step-loss tests** to measure the improvement from open-loop to
closed-loop control.

## Scope and descoping rationale

The original plan was a full position-repeatability test (ISO 230-2). Without a repeatable method for
sub-mm carriage measurements, and given the significant environmental and hardware constraints, the
testing was **descoped to acceleration-handling only** — which is also the only metric needed to
quantify a performance improvement for a project like an inverted pendulum.

Sub-mm position repeatability requires external measurement hardware, is messy to set up, and doesn't
really inform pendulum performance. This is a linear controls platform, not a pick-and-place or CNC
machine — so acceleration / step-loss testing is the practical measurement actually needed.

This plan will be carried out again once the pendulum payload is fixed; the **first tests are
load-free** to standardize the testing. A more robust testing suite (and its potential issues) should
be written up alongside the descoping rationale for future reference.

## What we're measuring: A_OL and A_CL

Two acceleration limits, one per control mode:

- **A_OL** — the acceleration at which **open-loop** control accumulates error against the encoder.
  Error is defined by the **open-loop position-mismatch threshold**.
- **A_CL** — the acceleration at which **closed-loop** control accumulates error. (On a velocity
  ramp — closed-loop is expected to eventually reach the target position in essentially every case.)
  Error is likewise defined by the **open-loop position-mismatch threshold — NOT** the closed-loop
  servo deadband, so both modes are judged against the same yardstick.

## Procedure

1. **Warm-up:** an unrecorded open-loop run, at least 1 minute of runtime.
2. **Acceleration ramp:** 20 moves (10 back-and-forth pairs) of a trapezoidal velocity profile
   (roughly 1/3 accelerate, cruise, decelerate — unless a more standard profile is found) with
   **increasing acceleration** move to move. Tune the ramp so the later moves trigger an open-loop
   position mismatch; closed-loop will likely need to be pushed much higher.
3. **Bisection:** once A_OL / A_CL are bracketed, bisect the previous run to find the precise
   acceleration limit.

## Data recording

- Use the standard serial telemetry as of 2026-07-12, **plus a timestamp** (see
  [`TELEMETRY.md`](TELEMETRY.md)).
- Drop the `key=value` format and condense the printing cost as much as possible.

## Data processing

MATLAB — plots and tables are the main artifacts.

## Firmware readiness (code modifications, 2026-07-13)

These changes prepare the firmware for the test code (the test code itself is written separately):

- **Microstep selection cleanup** — microstep mode and steps-per-rev are tied together in one place
  so they can't be set inconsistently. Implemented via the `ACTIVE_MICROSTEP` config in
  `src/LR_MS2_BaseCode.cpp`; see [`../hardware/MICROSTEPPING.md`](../hardware/MICROSTEPPING.md).
- **Consolidated speed/acceleration parameters** — the positioning speed and acceleration live in one
  `MOTION TUNING` block, kept independent of the control mode, with `setMotionSpeed()` /
  `setMotionAccel()` runtime setters so the acceleration ramp above can be swept without editing
  `setup()` or the state machine.
- **Condensed telemetry** — values-only CSV with a leading `millis()` timestamp and a one-time boot
  header line for MATLAB import; documented in [`TELEMETRY.md`](TELEMETRY.md).

## Test methodology note (open- vs. closed-loop)

Both modes are kept in one firmware image, toggled by the compile-time `controlMode` global (open vs.
closed loop), rather than split across git branches. Running the two modes back-to-back (reflashing
between them) keeps every comparison adjacent, so hardware drift — wear, belt tension, power supply —
applies to both modes rather than to only one batch of runs.
