# Test-Run Procedure — Capturing Telemetry CSVs

Copy-paste terminal template for running the acceleration/step-loss battery and saving one CSV per
run. Runs in the **VSCode integrated terminal (PowerShell)**. No Claude Code session required — these
are commands you run yourself between compile-time parameter changes.

- Data format the CSVs follow: [`MATLAB_DATA_HANDOFF.md`](MATLAB_DATA_HANDOFF.md).
- What to sweep and why: [`TEST_BATTERY.md`](TEST_BATTERY.md).

---

## 0. One-time setup — put `pio` on PATH

`pio` is installed under PlatformIO's private environment and is **not on PATH by default**, so a bare
`pio …` in the terminal fails with *"'pio' is not recognized"*. Fix it once.

**Permanent (recommended) — run once, then open a NEW terminal:**
```powershell
$scripts = "$HOME\.platformio\penv\Scripts"
$userPath = [Environment]::GetEnvironmentVariable("Path","User")
if ($userPath -notlike "*$scripts*") { [Environment]::SetEnvironmentVariable("Path", "$userPath;$scripts", "User") }
```

**Or per-session (this terminal only) — run at the top of each new terminal:**
```powershell
$env:Path += ";$HOME\.platformio\penv\Scripts"
```

Verify:
```powershell
pio --version
```

---

## Per-run loop

Repeat for every configuration (each control mode × microstep × sweep). **Do not run the monitor and
the upload at the same time** — the monitor holds the COM port, so an upload will fail until you stop
it (Ctrl+C).

### 1. Set the compile-time parameters
In `src/LR_MS2_BaseCode.cpp`, set for this run and **save**:
- `controlMode` → `ControlMode::OPEN_LOOP` or `ControlMode::CLOSED_LOOP`
- `ACTIVE_MICROSTEP` → `MS_HALF` / `MS_QUARTER` / `MS_EIGHTH`
- sweep constants as needed: `TEST_CRUISE_SPEED_HZ`, `TEST_ACCEL_START`, `TEST_ACCEL_STEP`

### 2. Flash the board
```powershell
pio run -t upload
```

### 3. Start logging to a named file
Name the file per the convention (§ below). The monitor prints live **and** writes the file:
```powershell
pio device monitor -b 115200 -p COM6 | Tee-Object -FilePath 20260720_ol_ms4_cruise6000_a2000-21000_s1000_bracket_r1.csv
```
> `-p COM6` is this board's port. Drop it to auto-detect, or run `pio device list` if it changes.

### 4. Reset the board to capture a clean header
Press the ESP32's **EN/RST** button now. The `#` column header prints **once at boot**, so resetting
*after* the monitor is attached guarantees the file starts with a fresh header, the homing sequence,
and `t` near 0.

### 5. Run the battery
Wait for homing to finish (rail reaches **IDLE**), then press the **test-start button (GPIO 25)** to
start the ramp.

### 6. Stop the capture
When the run returns to IDLE (you'll see `phase=DONE`, then `mode=IDLE`), press **Ctrl+C** in the
terminal. This detaches the monitor, flushes and closes the file, and frees the port.
> Always stop with **Ctrl+C** — don't close the terminal window mid-run, or the file may not flush.

### 7. Sanity-check the file
```powershell
Get-Content .\20260720_ol_ms4_cruise6000_a2000-21000_s1000_bracket_r1.csv -TotalCount 3
```
Expect the `# t,ctrl,mode,phase,err,…` header on line 1 and data rows after it.

### 8. Repeat
Back to step 1 with the next configuration and a **new filename**.

---

## Filename convention

The CSV cannot reconstruct microstep, cruise speed, or the accel sweep — those live **only in the
filename**. Fixed, `_`-delimited field order:

```
YYYYMMDD_<ol|cl>_ms<2|4|8>_cruise<hz>_a<start>-<top>_s<step>_<bracket|bisect>_r<n>.csv
```
Example: `20260720_ol_ms4_cruise6000_a2000-21000_s1000_bracket_r1.csv`

| Field | Example | Notes |
|---|---|---|
| date | `20260720` | YYYYMMDD |
| control | `ol` / `cl` | matches the `ctrl` column (cross-check) |
| microstep | `ms4` | 1/2, 1/4, 1/8 → sets `STEPS_PER_MM` — **filename only** |
| cruise | `cruise6000` | steps/s — **filename only** |
| accel sweep | `a2000-21000` | start→top, steps/s² |
| step | `s1000` | per-move accel increment, steps/s² |
| stage | `bracket` / `bisect` | coarse vs. fine run |
| repeat | `r1` | repeat index |

Optional `<same-stem>.txt` sidecar for anything else (load, belt tension, firmware commit, a
non-default mismatch tolerance).

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `'pio' is not recognized` | PATH not set — run the § 0 setup in this terminal (or the permanent one, then reopen the terminal). |
| Upload fails, port busy / access denied | A monitor is still holding the port. Press **Ctrl+C** in that terminal first, then upload. |
| CSV has no `#` header on line 1 | Monitor was started after boot. Reset (EN/RST) *after* the monitor is attached (step 4). |
| Wrong / no serial data | Wrong COM port. Run `pio device list`, then set `-p COMx` in step 3. |
| A `#` header appears mid-file and `t` jumps back to ~0 | The board reset mid-capture — the file now holds two segments. Note it; it's a known case (see the hand-off doc). |

---

*Want a PDF? Open this file in VSCode and use the built-in Markdown preview → print, or any Markdown-to-PDF
extension. Keeping the master copy as Markdown in the repo means the commands stay copy-pasteable and
versioned with the firmware.*
