# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

ESP32 (PlatformIO/Arduino) firmware for a fully 3D-printed, belt-driven linear rail. It's a platform for later controls demos, not a finished product. The rail homes against a limit switch, then supports manual jogging. An AS5600 magnetic encoder was recently integrated as an independent position-feedback verification signal (open-loop stepper counting is still what actually drives motion — see `docs/ARCHITECTURE.md`).

Repo layout: `src/` + `include/` (firmware), `lib/` (empty, PlatformIO placeholder — no private libraries yet), `test/` (empty, PlatformIO placeholder — no automated tests), `hardware/` (pinout/mechanical reference), `docs/` (architecture and calibration history).

## Build commands

This is a PlatformIO project (`platformio.ini`, single environment `env:esp32dev`, `board = esp32dev`, `framework = arduino`). **There is currently only one build environment** — no separate bench-test vs. on-rail environment exists. If bench testing (e.g. without the limit switch or encoder wired) is ever needed as a distinct build, that would require a new `[env:...]` section plus conditional (`#ifdef`)-guarded behavior in the firmware itself, not just a config split — worth a deliberate decision before adding, not something to assume exists.

`pio` is not necessarily on `PATH`. If `pio run` fails with "command not found", use the full path to the PlatformIO CLI installed under the user's penv, e.g. `~/.platformio/penv/Scripts/pio.exe` (Windows) — locate it with `ls ~/.platformio/penv/Scripts/` if unsure.

- Build: `pio run`
- Upload/flash: `pio run -t upload`
- Serial monitor (115200 baud): `pio device monitor -b 115200`
- Clean: `pio run -t clean`

There is no unit test suite — `test/` only contains PlatformIO's placeholder `README`. Verification is done on hardware via the serial monitor's telemetry line (see `printStatus()` below), not automated tests.

## Architecture

Two source files, both under `src/` (PlatformIO auto-compiles everything in `src/*.cpp` and adds `include/` to the include path — no build-script wiring needed when adding files):

- **`src/LR_MS2_BaseCode.cpp`** — the entire rail control state machine, single file.
- **`src/Encoder.cpp`** / **`include/Encoder.h`** — `RailEncoder`, a thin wrapper around the `robtillaart/AS5600` library + `Wire`, kept deliberately isolated so the state machine file never touches `AS5600`/`Wire` directly.

Full state-machine shape, the encoder's verification-only design decision, and its calibration history live in [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — read that before making non-trivial changes to `loop()` or the mismatch-detection logic. Pin assignments and mechanical constants (belt/pulley ratio, steps-per-mm, etc.) live in [`hardware/PINOUT.md`](hardware/PINOUT.md); the `const` declarations at the top of `LR_MS2_BaseCode.cpp` are the source of truth if the two ever disagree.

### Control loop timing constraint

Step pulses are generated in hardware by `FastAccelStepper` (ESP32 RMT/MCPWM), so pulse timing is **independent of `loop()` timing** — `loop()` does not service motion every iteration; it issues a command (`commandVelocity`/`commandMoveTo`/`commandHalt`) only when intent changes, and hardware runs the move. This replaced an earlier `AccelStepper` design that toggled the STEP pin from `loop()`, where telemetry formatting/serial writes on the same thread caused a print-synced motion stutter. Do not reintroduce per-iteration software stepping.

Two internal throttles remain, and should stay *inside the callee* (not ad-hoc `millis()` checks scattered through `loop()`): `RailEncoder::update()` self-throttles to a 10ms poll interval because the AS5600 read is a blocking I2C transaction (up to `Wire.setTimeOut(5)` = 5ms), and `printStatus()` self-throttles to `PRINT_INTERVAL_MS` (a live-tuned placeholder) because `Serial.print` is slow. `Serial.setTxBufferSize(1024)` in `setup()` (before `Serial.begin`) keeps `Serial.print` non-blocking via a background UART TX buffer.

`POSITION_MISMATCH_TOLERANCE_MM` and `POSITION_MISMATCH_HALT_ENABLED` (declared near the top of `LR_MS2_BaseCode.cpp`) are **live calibration constants** currently being tuned against real hardware, not fixed forever — check their current values before assuming the mismatch halt is armed.

### Jog input handling

In `Mode::JOGGING`, the "both jog buttons pressed" case is checked **first** and stops the carriage (`commandVelocity(0)`) — a deliberate safety stop. (It was formerly below the single-button checks and thus unreachable dead code; it has since been moved to the top of the if-chain.)
