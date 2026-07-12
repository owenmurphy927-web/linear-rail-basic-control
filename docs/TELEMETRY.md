# Serial telemetry format

`printStatus()` (in `src/LR_MS2_BaseCode.cpp`) prints one self-describing `key=value`
line over the serial monitor (115200 baud), throttled to `PRINT_INTERVAL_MS`. This file
is the legend for those fields. **The `snprintf` format string in `printStatus()` is the
source of truth** — if you add/rename/reorder a field there, update this table too.

Example line (illustrative values):

```
ctrl=OL mode=JOGGING phase=- err=- pos=45.32 vsps=1600.0 vmms=64.00 enc=45.28 dpos=0.040 dmax=0.112 sp=111.30 cerr=66.02 mis=0 econn=1 eerr=0 lim=00 led=010
```

## Sign / frame conventions

- All positions (`pos`, `enc`, `sp`) are in **mm from the homed zero**.
- **Positive = away from the home switch; negative = toward it** (matches the stepper's
  direction convention and the `AS5600_COUNTERCLOCK_WISE` encoder setting).
- `STEPS_PER_MM = 25`, so a step count converts to mm by `/25`.

## Fields

| Field | Type / unit | Meaning |
|---|---|---|
| `ctrl`  | `OL` \| `CL` | Active control mode — Open-Loop (step count drives motion) or Closed-Loop (encoder feedback drives motion). Set by the `controlMode` compile-time global. |
| `mode`  | enum | Top-level state machine mode. See decode below. |
| `phase` | enum \| `-` | Homing sub-phase; `-` unless `mode=HOMING`. See decode below. |
| `err`   | enum \| `-` | Error cause; `-` unless `mode=ERROR`. See decode below. |
| `pos`   | mm | **Open-loop position** = stepper step count ÷ `STEPS_PER_MM`. The authority on position in open-loop mode. |
| `vsps`  | steps/s | Current stepper speed (signed) from `getCurrentSpeedInMilliHz()`. |
| `vmms`  | mm/s | Same velocity as `vsps`, converted to mm/s (`vsps ÷ STEPS_PER_MM`). |
| `enc`   | mm | **Encoder position** from the AS5600 (`railEncoder.positionMm()`). The real physical position. |
| `dpos`  | mm | Position **diff** = `pos − enc` (step count minus encoder). Open-loop drift indicator; positive means the step count reads ahead of the encoder. |
| `dmax`  | mm | Running **max** of `|dpos|` since the last home. Reset in `HomingPhase::SET_ZERO`. Tracks the worst step-vs-encoder disagreement seen. |
| `sp`    | mm | Closed-loop **setpoint** (`clSetpointMm`) — the position the carriage is commanded to reach/hold. Maintained in both modes for A/B logging; only *acted on* in closed-loop. |
| `cerr`  | mm | Closed-loop servo **error** = `sp − enc` (setpoint minus encoder). What `closedLoopServoTo` drives toward zero. In open-loop it's informational (how far the true position is from the same target). |
| `mis`   | 0/1 | Position **mismatch** flag: `1` when `|dpos| > POSITION_MISMATCH_TOLERANCE_MM`. Observe-only unless `POSITION_MISMATCH_HALT_ENABLED` is true. |
| `econn` | 0/1 | Encoder **connected**: `1` when the AS5600 last I2C read succeeded. `0` latches `ENCODER_NOT_DETECTED`. |
| `eerr`  | int | Encoder **last error** code from the AS5600 library (`railEncoder.lastError()`); `0` = `AS5600_OK`. |
| `lim`   | 2 digits `NF` | End-of-travel switches, each `1`=pressed. **N** = near/home switch (`HOMING_PIN`), **F** = far switch (`FAR_LIMIT_PIN`). Both wired NC, so `HIGH`/`1` = pressed *or* a broken/disconnected lead (fail-safe). |
| `led`   | 3 digits `HIE` | Status LED states, each `1`=on. **H** = HOME LED (blue), **I** = IDLE LED (green), **E** = ERROR LED. |

## Enum decodes

`ctrl` — `controlModeText()`:

| Value | Meaning |
|---|---|
| `OL` | Open-loop (default) |
| `CL` | Closed-loop |

`mode` — `modeText()`:

| Value | Meaning |
|---|---|
| `HOMING`  | Running the homing sequence |
| `IDLE`    | Homed, stopped, holding position |
| `JOGGING` | Manual jog via the buttons |
| `ERROR`   | Latched fault (terminal — needs reset/re-home) |

`phase` — `homingPhaseText()` (only meaningful when `mode=HOMING`):

| Value | Meaning |
|---|---|
| `FAST`   | Fast approach toward the home switch |
| `BACK`   | Back off after first touch |
| `SLOW`   | Slow re-approach for an accurate trigger point |
| `SETTLE` | Wait for `forceStop` to drain before repositioning |
| `ZERO`   | Pull off to zero and zero both position sources |
| `DONE`   | Brief LED confirmation before handing off to `IDLE` |

`err` — `errorModeText()` (only meaningful when `mode=ERROR`):

| Value | Meaning |
|---|---|
| `OK`       | No error |
| `TIMEOUT`  | Homing exceeded `HOMING_TIMEOUT` |
| `MISMATCH` | `POSITION_MISMATCH` (step-vs-encoder past tolerance, halt armed) |
| `NO_ENC`   | `ENCODER_NOT_DETECTED` (AS5600 dropped off I2C / failed to init) |
| `OVERTRVL` | `OVER_TRAVEL` (an end-of-travel switch tripped outside homing) |
| `STP_FAIL` | `STEPPER_INIT_FAILED` (FastAccelStepper init failed at boot) |
| `UNK`      | `UNKNOWN` |
