# Hardware / Pinout

ESP32 dev board (`esp32dev`) driving a belt-and-pulley linear rail through a STEP/DIR stepper driver, with two end-of-travel limit switches (home/near end used for homing, plus a far end), two manual jog buttons, three status LEDs, and an AS5600 magnetic encoder on I2C for position-feedback verification.

## Pin assignments

| Pin | Signal | Notes |
|---|---|---|
| 26 | `STEP_PIN` | Stepper driver STEP — step pulses generated in hardware by FastAccelStepper (ESP32 RMT/MCPWM), not bit-banged from `loop()`. |
| 27 | `DIR_PIN` | Stepper driver DIR — direction sense set via `setDirectionPin(DIR_PIN, false)` (inverted; verify on bench). |
| 23 | `HOMING_PIN` | Home limit switch (near-end limit) — wired NC, so HIGH = pressed, LOW = released. `INPUT_PULLUP`. Used for homing; also acts as the near-end over-travel limit outside homing. |
| 13 | `FAR_LIMIT_PIN` | Far end-of-travel limit switch — wired NC, so HIGH = pressed. `INPUT_PULLUP`. Over-travel failsafe only (not used during homing). |
| 32 | `MS1_PIN` | Microstep select 1 |
| 33 | `MS2_PIN` | Microstep select 2 (driver held at 1/4-step: MS1=LOW, MS2=HIGH) |
| 15 | `POS_JOG_PIN` | Positive-direction jog button, active LOW. `INPUT_PULLUP`. |
| 16 | `NEG_JOG_PIN` | Negative-direction jog button, active LOW. `INPUT_PULLUP`. |
| 17 | `ERROR_LED_PIN1` | Error indicator LED |
| 18 | `IDLE_LED_PIN` | Idle indicator LED (green) |
| 19 | `HOME_LED_PIN` | Homing-finished indicator LED (blue) |
| 21 | `ENCODER_SDA_PIN` | AS5600 I2C SDA |
| 22 | `ENCODER_SCL_PIN` | AS5600 I2C SCL |

No pin conflicts between the stepper/switch/LED/jog set and the encoder's I2C pair. `FAR_LIMIT_PIN` (GPIO 13) is a general-purpose input — not a strapping pin and supports the internal pull-up.

## Mechanical constants

- `REV_STEPS = 800` — steps per motor revolution at the driver's configured 1/4-microstep setting.
- `PULLEY_TEETH = 16`, `BELT_PITCH_MM = 2.0` → `MM_PER_REV = 32mm` of carriage travel per pulley revolution.
- `STEPS_PER_MM = REV_STEPS / MM_PER_REV = 25 steps/mm`.
- `HOME_PULL_OFF_MM = 5.0` — distance backed off the limit switch before the final slow re-approach during homing. Also places the near-switch trigger point at `-HOME_PULL_OFF_MM` relative to zero.
- `FAR_LIMIT_POSITION_MM = 116.3` — far switch trigger point, measured once from zero (not homed to). Full travel = `FAR_LIMIT_POSITION_MM + HOME_PULL_OFF_MM = 121.3mm`.
- `SOFT_LIMIT_BUFFER_MM = 5.0` — jog soft-limit keep-out off each switch. Jog bounds: `JOG_MIN_POSITION_MM = -HOME_PULL_OFF_MM + SOFT_LIMIT_BUFFER_MM = 0mm`, `JOG_MAX_POSITION_MM = FAR_LIMIT_POSITION_MM - SOFT_LIMIT_BUFFER_MM = 111.3mm`.

## Encoder mounting

The AS5600 magnet is mounted 1:1 on the drive pulley shaft, so one encoder revolution corresponds to exactly one `MM_PER_REV` (32mm) of carriage travel — this is what lets the encoder's raw tick count convert directly to mm without a separate gear ratio. Encoder direction is configured `AS5600_COUNTERCLOCK_WISE` in `railEncoder.begin(...)` (`src/LR_MS2_BaseCode.cpp`, `setup()`) to match the stepper's positive-direction sign convention — this was flipped from the library default after bring-up testing showed the two positions moving in opposite directions (see `docs/ARCHITECTURE.md` for the calibration history).

Source of truth for all values above is the `const` declarations near the top of `src/LR_MS2_BaseCode.cpp` — this file is a convenience summary; if the two ever disagree, the code wins.
