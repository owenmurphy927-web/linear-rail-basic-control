# linear-rail-basic-control

Firmware for a fully 3D-printed, belt-driven linear rail, built as a platform for later controls demos. This documents the code and current state of development — it does not cover the full project timeline.

## Hardware summary

An ESP32 dev board drives a stepper (via a STEP/DIR driver) through a toothed pulley and belt, with a limit switch for homing, two jog buttons, three status LEDs, and an AS5600 magnetic encoder on I2C for position-feedback verification. Full pin assignments and mechanical constants are in [`hardware/PINOUT.md`](hardware/PINOUT.md).

## Build instructions

This is a PlatformIO project (single environment, `esp32dev`).

- Build: `pio run`
- Upload/flash: `pio run -t upload`
- Serial monitor: `pio device monitor -b 115200`
- Clean: `pio run -t clean`

If `pio` isn't on `PATH`, use the full path to the PlatformIO CLI under the PlatformIO Core install (e.g. `~/.platformio/penv/Scripts/pio.exe` on Windows).

## Current implementation status

- **Homing, jogging, open-loop motion**: implemented. The stepper's own step count is the sole authority on carriage position.
- **Encoder position feedback**: implemented, as a verification/stall-detection layer only — it compares the encoder's physical reading against the open-loop step count and can halt into an error state on disagreement or disconnection. It does **not** drive motion; it doesn't feed back into control.
- **Closed-loop position control**: not implemented, and not currently in progress. The encoder work was scoped specifically as verification-only; using it to actually correct motion would be a separate, later effort.

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the state machine design and the encoder's calibration history, and [`CLAUDE.md`](CLAUDE.md) for repository-level notes aimed at AI coding assistants.
