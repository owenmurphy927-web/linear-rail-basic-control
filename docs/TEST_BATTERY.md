# Test Battery — Acceleration / Step-Loss

Consolidated reference for running the MS2 acceleration battery on the rail. It pulls together the
scattered planning notes so an explicit run matrix can be authored from one place. Sources:
[`TESTING_PLAN.md`](TESTING_PLAN.md) (goals/procedure), [`TESTING_OBSERVATIONS.md`](TESTING_OBSERVATIONS.md)
(informal brackets + the vibration confounder), [`TELEMETRY.md`](TELEMETRY.md) (CSV schema),
[`ARCHITECTURE.md`](ARCHITECTURE.md) (state machine / closed-loop law),
[`../hardware/MICROSTEPPING.md`](../hardware/MICROSTEPPING.md), and
[`../hardware/PINOUT.md`](../hardware/PINOUT.md). The firmware runner is `Mode::TESTING` in
`src/LR_MS2_BaseCode.cpp`.

## Objective & metrics

Measure the acceleration each control mode can handle before it loses steps, to quantify the
open-loop → closed-loop improvement (the metric that matters for a downstream inverted-pendulum
payload). Two numbers:

- **A_OL** — acceleration at which **open-loop** control accumulates step-vs-encoder error.
- **A_CL** — acceleration at which **closed-loop** control accumulates error. (On a velocity ramp CL
  is expected to reach the target in essentially every case, so A_CL should be much higher.)

**Both** are judged against the **open-loop position-mismatch threshold** — `POSITION_MISMATCH_TOLERANCE_MM`
(0.5 mm), **not** the CL servo deadband — so the two modes share one yardstick. In telemetry the
condition is the `mis` flag (`1` when `|dpos| > tolerance`); the raw evidence is `dpos` (`pos − enc`)
and its running max `dmax`.

## How the firmware runs it (`Mode::TESTING`)

Press the **test-start button (GPIO 25**, normally-open to GND) while the rail is `IDLE` (i.e. homed).
The sequence, a non-blocking sub-state machine, then:

1. **Warm-up** (`phase=WARM`) — gentle back-and-forth for `TEST_WARMUP_DURATION_MS` (≥1 min). Rows are
   tagged so they can be dropped in post. Skipped if `TEST_WARMUP_ENABLED = false`.
2. **Ramp** — `TEST_NUM_MOVES` (20) moves between `TEST_NEAR_MM` and `TEST_FAR_MM` at a fixed cruise
   speed `TEST_CRUISE_SPEED_HZ`, `phase` cycling `MOVE`→`SET` per move. Acceleration increases
   **linearly, per move**: `accel(i) = TEST_ACCEL_START + i·TEST_ACCEL_STEP`. Each move's accel is
   logged in the `accel` column, so every move is one accel level.
3. **Finish** — returns to `IDLE` and holds.

Logging runs at `TEST_PRINT_INTERVAL_MS` (50 Hz) during the test vs. 10 Hz normally. Safety stays
live: the over-travel failsafe still latches `ERROR` on a switch trip, encoder disconnect still
faults, and **pressing both jog buttons aborts** the run back to `IDLE`. The mismatch check is
observe-only (`POSITION_MISMATCH_HALT_ENABLED = false`), so a run isn't cut short when error is
detected — you see `mis` go to 1 and keep the data.

### Procedure per the plan
- **Bracket first:** a wide sweep (large `TEST_ACCEL_STEP`, or set START/STEP to straddle the
  empirical brackets below) until the later moves trip `mis`.
- **Then bisect:** narrow `TEST_ACCEL_START` around the bracket and shrink `TEST_ACCEL_STEP` to pin
  A_OL / A_CL precisely.
- **One image, toggle + reflash:** control mode (`controlMode`) and microstep (`ACTIVE_MICROSTEP`) are
  compile-time. Run OL and CL back-to-back (reflashing between) so hardware drift — belt tension,
  wear, supply — applies to both. Repeat per microstep mode.

## Parameter glossary

All in `src/LR_MS2_BaseCode.cpp`. Speeds/accels are steps/s (/s²); mm equivalents use the active
`STEPS_PER_MM`.

| Parameter | Current | Meaning / how to sweep |
|---|---|---|
| `TEST_CRUISE_SPEED_HZ` | 6000 | Fixed cruise speed for every ramp move. Hold constant within a battery; the test varies accel, not speed. |
| `TEST_ACCEL_START` | 2000 | Acceleration of move 0. Lower bound of the sweep. |
| `TEST_ACCEL_STEP` | 1000 | Per-move accel increment. Wide for bracketing, small for bisection. |
| `TEST_NUM_MOVES` | 20 | Move count (10 back/forth). Top accel = START + (N−1)·STEP. |
| `TEST_NEAR_MM` / `TEST_FAR_MM` | `JOG_MIN_POSITION_MM` / `JOG_MAX_POSITION_MM` (≈3.0 / 108.3 mm at `SOFT_LIMIT_BUFFER_MM = 8`) | Travel endpoints. Default is the full safe span. |
| `TEST_SETTLE_MS` | 400 | Dwell after each move fully stops (clean settled sample). |
| `TEST_WARMUP_ENABLED` | true | Include the ≥1 min warm-up. |
| `TEST_WARMUP_DURATION_MS` | 60000 | Warm-up length. |
| `TEST_WARMUP_ACCEL_HZ2` / `TEST_WARMUP_SPEED_HZ` | 4000 / 4000 | Gentle warm-up motion (kept below the sweep). |
| `TEST_PRINT_INTERVAL_MS` | 20 | Telemetry period while testing (50 Hz). |
| `controlMode` | `OPEN_LOOP` | Compile-time OL/CL toggle → `ctrl` column. |
| `ACTIVE_MICROSTEP` | `MS_QUARTER` | Compile-time microstep (drives pins + `REV_STEPS` + `STEPS_PER_MM`). |
| `POSITION_MISMATCH_TOLERANCE_MM` | 0.5 | The pass/fail yardstick for A_OL and A_CL (`mis` flag). |
| `POSITION_MISMATCH_HALT_ENABLED` | false | Keep **false** for testing (observe, don't halt). |
| `motionAccelStepsPerS2` | 12000 | Live commanded accel; driven by the ramp via `setMotionAccel()`, logged as `accel`. |
| `CL_KP`, `CL_DEADBAND_MM`, `CL_MAX_CORRECTION_MM`, `CL_TRAVEL_MAX_CORRECTION_MM`, `CL_UPDATE_INTERVAL_MS` | 0.8, 0.10, 8.0, 120.0, 20 | Closed-loop servo gains/limits (see ARCHITECTURE.md). Relevant only in CL runs. |

## Microstep matrix

Change `ACTIVE_MICROSTEP` (one line) and reflash. `STEPS_PER_MM = stepsPerRev / 32`.

| Mode | `ACTIVE_MICROSTEP` | steps/rev | `STEPS_PER_MM` | In battery? |
|---|---|---|---|---|
| 1/2  | `MS_HALF`      | 400  | 12.5 | yes |
| 1/4  | `MS_QUARTER`   | 800  | 25   | yes (current) |
| 1/8  | `MS_EIGHTH`    | 1600 | 50   | yes |
| 1/16 | `MS_SIXTEENTH` | 3200 | 100  | **excluded** per plan |

Note: finer microstepping is also the biggest single anti-resonance lever (see confounder below), so
expect the vibration character — and possibly A_OL/A_CL — to shift across these modes.

## Empirical brackets (informal, 1/4-step — NOT test data)

From `TESTING_OBSERVATIONS.md`, load-free, 1/4-step. Use only to set the sweep range, not as results:

| Mode | Speed (steps/s) | Accel (steps/s²) | Result |
|---|---|---|---|
| CL | 6000  | 1500  | Clean, repeatable (vibration eased by belt tension) |
| CL | 8000  | 20000 | Major vibration; likely desync/stall |
| OL | 6000  | 10000 | Clean, no step-loss |
| OL | 10000 | 12000 | **Decoupled**, drove into the far limit |

→ Starting points: OL sweep should straddle ~10000 steps/s² (clean) up toward where run 4 failed; CL
tolerates far higher accel and will need to be pushed well past 20000 to find A_CL.

## Data capture & MATLAB

The stream is already MATLAB-ready CSV (comma-delimited, one `#` header at boot). Prefer **capturing
to a file** over hand copy/paste from the monitor:

```powershell
pio device monitor -b 115200 | Tee-Object -FilePath run_ol_ms4.csv
```

(or any serial logger). Then in MATLAB:

- `readmatrix`/`readtable` — the leading `#` header line and the `ctrl`/`mode`/`phase`/`err` string
  columns are handled by `readtable`; `readmatrix` will skip/NaN the text columns.
- **Segment by the `accel` column** — each distinct value is one ramp move / accel level.
- **Drop warm-up:** filter out rows where `phase == "WARM"`.
- **Key metrics:** `dpos` (`pos−enc`), `dmax`, `mis`, `accel`, plus `enc`/`pos`/`vsps`/`vmms`.
- A_OL / A_CL = the lowest `accel` level whose move shows `mis==1` (or per-move max `|dpos|` > 0.5 mm).
- **Packed flag columns:** `lim` (2-digit NF) and `led` (3-digit HIE) import as plain integers
  (e.g. `010` → 10); decode digit-wise only if you need the individual switch/LED states — they
  aren't core to the accel metric.

See [`TELEMETRY.md`](TELEMETRY.md) for the full column legend.

## Known confounder: mid-span vibration

Both modes show vibration worst at **mid-span + high speed**, eased by belt tension, independent of
table coupling — so it's mechanical (belt/motor), not control-law. Leading cause: belt compliance is
lowest at mid-span → lowest natural frequency; stepper mid-band resonance likely drives the
outright desync/stall. `FastAccelStepper` does **linear acceleration only (no jerk/S-curve limit)**, so
accel steps are impulsive and excite these modes — relevant when interpreting A_OL/A_CL. Levers:
repeatable belt tension, finer microstepping (1/8), a better driver (TMC2209), avoiding the resonant
speed band. A hardware upgrade is under consideration before extensive testing — revisit the matrix
after. Full analysis in `TESTING_OBSERVATIONS.md`.

## Run matrix template

Fill one row per run (reflash between control modes and microsteps):

| Date | ctrl | microstep | cruise (steps/s) | accel range (start→top, steps/s²) | step | pass (bracket/bisect) | A_OL or A_CL (steps/s² · mm/s²) | log file | notes |
|---|---|---|---|---|---|---|---|---|---|
|  | OL | 1/4 | 6000 | 2000 → 21000 | 1000 | bracket |  |  |  |
|  | CL | 1/4 | 6000 |  |  | bracket |  |  |  |
|  | … |  |  |  |  |  |  |  |  |
