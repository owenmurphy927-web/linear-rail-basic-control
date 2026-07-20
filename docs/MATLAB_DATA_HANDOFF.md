# MATLAB Data Hand-off — Telemetry Reference

**Portable data dictionary for the linear-rail acceleration/step-loss CSV logs.**

This document is meant to be **copied out of the repo** and handed to a separate session that does the
MATLAB processing from local `.csv` files. It is self-contained: it describes only *what the data is*,
so a reader with no access to the firmware can load and reason about the logs. It intentionally
contains **no analysis instructions** — how to segment, filter, or compute metrics is supplied
separately. See the [Not covered here](#not-covered-here) list at the end.

---

## 1. What this data is

- Each file is a captured **serial log** from an ESP32 that controls a belt-driven linear rail.
- One line = **one telemetry sample**. Samples are emitted periodically while the firmware runs.
- The interesting files are **automated test runs** (the firmware's `TESTING` mode): the carriage
  drives back and forth while its commanded **acceleration is increased step by step**, to find the
  acceleration at which the rail loses steps. The rail is compared in two control modes, **open-loop
  (OL)** and **closed-loop (CL)**.
- The CSV carries **raw values with no units and no key names** — this document is the legend. Units,
  sign conventions, and one critical value that is *not in the file* (the microstep resolution) are
  all defined below.

---

## 2. File anatomy

- **Comma-delimited**, one sample per line, **values only** (no `key=value` prefixes).
- The **first line is a header row**, printed once when the board boots, prefixed with `#`:

  ```
  # t,ctrl,mode,phase,err,pos,vsps,vmms,enc,dpos,dmax,sp,cerr,mis,econn,eerr,lim,led,accel
  ```

  There are **19 columns** in a fixed order (detailed in §3). The `#` marks it as a comment/header.
- Columns are **mixed string and numeric**: `ctrl`, `mode`, `phase`, `err` are short text tokens; the
  rest are numbers. A **text-aware reader is required** (e.g. `readtable`); a purely numeric read
  (e.g. `readmatrix`) will turn the four text columns into `NaN`.
- **Example data line** (illustrative values):

  ```
  10432,OL,JOGGING,-,-,45.32,1600.0,64.00,45.28,0.040,0.112,111.30,66.02,0,1,0,00,010,12000
  ```

- **Board reset mid-capture (multi-segment files):** if the board resets while logging, a **second
  `#` header line** appears partway through the file and the timestamp column `t` **jumps back toward
  0** (it is a since-boot counter — see §4). A clean single run has exactly one header at the top and
  a monotonically increasing `t`.

---

## 3. Column dictionary (all 19 fields, in order)

| # | Field | Type / unit | Meaning |
|---|---|---|---|
| 1 | `t`     | ms | **Timestamp** — milliseconds since the board booted. The time base for plots. Monotonic within one boot; see §4 for wrap/reset behavior. |
| 2 | `ctrl`  | text `OL`\|`CL` | **Control mode** active for this run — Open-Loop (step count drives motion) or Closed-Loop (encoder feedback drives motion). Constant within a run. |
| 3 | `mode`  | text enum | **State-machine mode**: `HOMING`, `IDLE`, `JOGGING`, `TESTING`, `ERROR`. Test-run data is the `TESTING` rows. |
| 4 | `phase` | text enum \| `-` | **Sub-phase**. Meaningful when `mode=HOMING` or `mode=TESTING`; `-` otherwise. Test values: `WARM`, `MOVE`, `SET`, `DONE` (see §3.1). |
| 5 | `err`   | text enum \| `-` | **Error cause**; `-` unless `mode=ERROR`. See §3.1. |
| 6 | `pos`   | mm | **Open-loop position** — derived from the stepper's commanded step count. The controller's *belief* about position. |
| 7 | `vsps`  | steps/s | Current stepper **velocity**, signed. |
| 8 | `vmms`  | mm/s | Same velocity converted to mm/s. |
| 9 | `enc`   | mm | **Encoder position** — the AS5600 magnetic encoder reading. The *measured physical* position. |
| 10 | `dpos` | mm | **Position difference** = `pos − enc` (belief minus measurement). The open-loop drift/step-loss indicator; positive means the step count reads ahead of the encoder. |
| 11 | `dmax` | mm | **Running max of `|dpos|`** since the last home. Reset at homing; tracks the worst step-vs-encoder disagreement seen so far in the run. |
| 12 | `sp`   | mm | **Setpoint** — the position the carriage is commanded to reach/hold. Logged in both control modes (so OL and CL runs record the same target); only *acted on* in closed-loop. |
| 13 | `cerr` | mm | **Servo error** = `sp − enc` (setpoint minus measured position). In CL this is what the servo drives toward zero; in OL it is informational. |
| 14 | `mis`  | 0/1 | **Mismatch flag** — `1` when `|dpos|` exceeds the firmware's mismatch tolerance (0.5 mm; see §4). This is a *derived* flag, computed on the device. |
| 15 | `econn`| 0/1 | **Encoder connected** — `1` when the last encoder I²C read succeeded, `0` if it dropped off the bus. |
| 16 | `eerr` | int | **Encoder last-error code** from the AS5600 library; `0` = OK. |
| 17 | `lim`  | 2-digit `NF` | **End-of-travel switches**, packed: **N** = near/home switch, **F** = far switch; each digit `1`=pressed. See §5 for the leading-zero import trap. |
| 18 | `led`  | 3-digit `HIE` | **Status LEDs**, packed: **H** = home, **I** = idle, **E** = error; each digit `1`=on. See §5. |
| 19 | `accel`| steps/s² | **Commanded acceleration** for the current move. During a test ramp this steps up once per move — it is the independent variable of the experiment. Convert to mm/s² by dividing by `STEPS_PER_MM` (see §4 — **not in the file**). |

### 3.1 Text-column enumerations

**`ctrl`** — control mode:

| Value | Meaning |
|---|---|
| `OL` | Open-loop (step count drives motion) |
| `CL` | Closed-loop (encoder feedback drives motion) |

**`mode`** — state-machine mode:

| Value | Meaning |
|---|---|
| `HOMING`  | Running the homing sequence (position sources not yet comparable) |
| `IDLE`    | Homed, stopped, holding position |
| `JOGGING` | Manual jog via buttons |
| `TESTING` | Running the automated acceleration/step-loss ramp |
| `ERROR`   | Latched fault (terminal — needs reset/re-home) |

**`phase`** when `mode=TESTING`:

| Value | Meaning |
|---|---|
| `WARM` | Warm-up oscillation before the recorded ramp |
| `MOVE` | Driving one ramp move at the current `accel` |
| `SET`  | Settling/holding at an endpoint between moves |
| `DONE` | Sequence complete; handing back to `IDLE` |

**`phase`** when `mode=HOMING`: `FAST`, `BACK`, `SLOW`, `SETTLE`, `ZERO`, `DONE` (stages of the homing
seek; not part of test data).

**`err`** when `mode=ERROR`:

| Value | Meaning |
|---|---|
| `OK`       | No error |
| `TIMEOUT`  | Homing exceeded its timeout |
| `MISMATCH` | Step-vs-encoder past tolerance (only latches if the halt is armed) |
| `NO_ENC`   | Encoder not detected / dropped off I²C |
| `OVERTRVL` | An end-of-travel switch tripped outside homing |
| `STP_FAIL` | Stepper driver failed to initialize at boot |
| `UNK`      | Unknown |

---

## 4. Sign, frame, and unit conventions

- **Positions** (`pos`, `enc`, `sp`) are in **mm from the homed zero**.
  **Positive = away from the home switch; negative = toward it.** `pos`, `enc`, and `sp` share this
  frame, so `dpos = pos − enc` and `cerr = sp − enc` are directly comparable.
- **Velocities**: `vsps` (steps/s) and `vmms` (mm/s) are **signed** with the same direction convention.
- **Time** (`t`) is `millis()` since boot: **monotonic within a boot**, wraps back to 0 after ~49 days
  (not a concern for minutes-long runs), and **resets on a board reset** (see the multi-segment note
  in §2).
- **Acceleration `accel` is in steps/s².** Converting it (or `vsps`, or `pos`) to millimetres requires
  **`STEPS_PER_MM`, which is _not present anywhere in the CSV_.** It is set by the compile-time
  microstep resolution:

  | Microstep | `STEPS_PER_MM` | mm per step |
  |---|---|---|
  | 1/2 step  | 12.5 | 0.080 |
  | 1/4 step  | 25   | 0.040 |
  | 1/8 step  | 50   | 0.020 |

  ⚠️ **This is the single most important thing to get from outside the file.** The microstep setting
  travels in the **filename** (`ms2` / `ms4` / `ms8` → 1/2, 1/4, 1/8), per §6. Without it, `accel` in
  mm/s² is undefined. (`pos`/`enc`/`vmms` are already in mm in the stream and don't need it — but
  `accel` and `vsps` do.)
- **Mismatch tolerance:** the `mis` flag (col 14) trips at **0.5 mm** of `|dpos|` in the standard
  firmware. This threshold lives in the firmware, not the file; it is stated here so the flag is
  interpretable. If a run used a non-default value it will be noted in that run's sidecar (§6).

---

## 5. Import gotchas

- **Packed flag columns lose leading zeros.** `lim` (2-digit `NF`) and `led` (3-digit `HIE`) are
  printed as fixed-width digit strings but **import as plain integers**, so `010` becomes `10` and
  `00` becomes `0`. To read individual switch/LED bits, zero-pad back to width (2 for `lim`, 3 for
  `led`) *before* splitting digits. These columns are auxiliary status, not core to the acceleration
  data.
- **`mis`, `econn`** are 0/1 integer flags. **`eerr`** is a small integer library error code
  (`0` = OK).
- **Text vs numeric:** as noted in §2, the four text columns require a text-aware reader or they
  import as `NaN`.
- **`-` placeholders:** `phase` and `err` contain the literal string `-` when not applicable; a
  numeric reader will treat these as missing/`NaN`.

---

## 6. How the files are named (context carried outside the CSV)

Each file is **one test run** (one control mode × one microstep × one acceleration sweep). Some run
parameters are **not in the stream** and are encoded in the filename instead. The convention is a
fixed, `_`-delimited field order:

```
YYYYMMDD_<ol|cl>_ms<2|4|8>_cruise<hz>_a<start>-<top>_s<step>_<bracket|bisect>_r<n>.csv
```

Example: `20260720_ol_ms4_cruise6000_a2000-21000_s1000_bracket_r1.csv`

| Filename field | Example | Meaning | Also in CSV? |
|---|---|---|---|
| date        | `20260720` | Run date (YYYYMMDD) | no |
| control     | `ol`/`cl`  | Control mode | yes — `ctrl` column (cross-check they agree) |
| microstep   | `ms4`      | 1/2, 1/4, 1/8 → sets `STEPS_PER_MM` (§4) | **no — filename only** |
| cruise      | `cruise6000` | Fixed cruise speed for every move, steps/s | **no — filename only** |
| accel sweep | `a2000-21000` | Acceleration start→top of the ramp, steps/s² | partially — per-sample value is in `accel` |
| step        | `s1000`    | Per-move acceleration increment, steps/s² | no (inferable from consecutive `accel` levels) |
| stage       | `bracket`/`bisect` | Coarse bracketing run vs. fine bisection run | no |
| repeat      | `r1`       | Repeat index | no |

An **optional sidecar** `<same-stem>.txt` may sit next to a CSV carrying anything not in the filename
(load/payload, belt tension, firmware commit, a non-default mismatch tolerance). When present it is
plain text notes for that one run.

---

## 7. Run shape (properties of the data, for orientation)

This describes what a typical `TESTING` file *looks like*, so the columns make sense. It is context,
not a processing recipe.

- A run may begin with a **warm-up segment**: gentle back-and-forth oscillation whose rows carry
  `phase=WARM`. It precedes the recorded ramp. (It can be disabled, so some files have none.)
- The **recorded ramp** is roughly **20 moves**, alternating between a near endpoint and a far
  endpoint. Within each move `phase` cycles `MOVE` (driving) → `SET` (settling at the endpoint).
- **`accel` increases once per move** and is constant during that move — so each distinct `accel`
  value corresponds to one move / one acceleration level. This is why `accel` looks like a staircase
  over time.
- The stepper's belief (`pos`) and the encoder (`enc`) track together until the rail starts losing
  steps; `dpos` (their difference) and `dmax` (its running max) are where that shows up, and the
  device-side `mis` flag marks samples past the 0.5 mm tolerance.
- **Sampling rate** is higher during `TESTING` (~50 Hz) than during ordinary operation (~10 Hz), so
  acceleration transients are resolved. Rate is not perfectly uniform — always use the `t` column as
  the true time base rather than assuming a fixed sample period.
- The run ends with `phase=DONE` and a return to `IDLE`.

---

## Not covered here

Deliberately **out of scope** for this reference — these are analysis choices supplied separately:

- Which rows to drop or keep (e.g. warm-up, settling, non-`TESTING` rows).
- How to segment the data into per-move / per-acceleration-level groups.
- The definition of the acceleration limit (A_OL / A_CL) or any pass/fail criterion.
- Which fields to plot, table formats, or any output preferences.
- Any judgement of one control mode against the other.

This document defines *only* the data; the processing intent comes from the user.
