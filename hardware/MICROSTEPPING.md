# Microstepping reference

Bench-measured map of the stepper driver's two microstep-select pins (`MS1_PIN` = GPIO 32,
`MS2_PIN` = GPIO 33) to microstep resolution and steps-per-revolution. This is the reference the
firmware's microstep config points back to.

## Pin → resolution table

| MS1  | MS2  | Microstep       | Steps/rev | Revolutions per 1600 steps commanded |
|------|------|-----------------|-----------|--------------------------------------|
| LOW  | LOW  | 1/8             | 1600      | 1                                    |
| HIGH | LOW  | 1/2             | 400       | 4                                    |
| LOW  | HIGH | **1/4** *(current)* | **800** | 2                                |
| HIGH | HIGH | 1/16            | 3200      | 0.5                                  |

## How this was measured

The motor is a standard 200-full-step/rev stepper. The bench method commanded a fixed **1600 steps**
in each pin configuration and counted the resulting output revolutions, which back out the
steps-per-rev in each mode:

- 1/8 step → 1600 steps/rev → 1600 commanded = **1 rev**
- 1/2 step → 400 steps/rev → 1600 commanded = **4 rev**
- 1/4 step → 800 steps/rev → 1600 commanded = **2 rev**
- 1/16 step → 3200 steps/rev → 1600 commanded = **0.5 rev**

Note the mapping is **not** a plain binary count on `(MS1, MS2)` — it is the empirically-observed
behavior of this specific driver, so trust the table above rather than a datasheet guess.

## Source of truth in code

`src/LR_MS2_BaseCode.cpp` encodes this table as a `constexpr MicrostepMode` set (`MS_HALF`,
`MS_QUARTER`, `MS_EIGHTH`, `MS_SIXTEENTH`). One selection, `ACTIVE_MICROSTEP`, drives **both** the
`MS1`/`MS2` pin states written in `setup()` **and** `REV_STEPS` (steps per rev), so the two can never
drift out of sync. `STEPS_PER_MM` derives from `REV_STEPS`, so changing `ACTIVE_MICROSTEP` is the
only edit needed to re-gear the whole position/velocity math.

The rail currently runs `ACTIVE_MICROSTEP = MS_QUARTER` (1/4 step, `REV_STEPS = 800`,
`STEPS_PER_MM = 25`), matching the highlighted row above. See [`PINOUT.md`](PINOUT.md) for the pin
assignments and mechanical constants.
