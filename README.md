# linear-rail-basic-control

Firmware for a fully 3D-printed, belt-driven linear rail, built as a platform for later controls demos. This documents the code and current state of development — it does not cover the full project timeline.

## Hardware summary

An ESP32 dev board drives a stepper (via a STEP/DIR driver) through a toothed pulley and belt, with a limit switch for homing, two jog buttons, three status LEDs, and an AS5600 magnetic encoder on I2C for position feedback (verification in open-loop mode, and the feedback signal in closed-loop mode). Full pin assignments and mechanical constants are in [`hardware/PINOUT.md`](hardware/PINOUT.md).

## Build instructions

This is a PlatformIO project (single environment, `esp32dev`).

- Build: `pio run`
- Upload/flash: `pio run -t upload`
- Serial monitor: `pio device monitor -b 115200`
- Clean: `pio run -t clean`

If `pio` isn't on `PATH`, use the full path to the PlatformIO CLI under the PlatformIO Core install (e.g. `~/.platformio/penv/Scripts/pio.exe` on Windows).

## Current implementation status

- **Homing, jogging, open-loop motion**: implemented. In open-loop mode the stepper's own step count is the sole authority on carriage position.
- **Encoder position feedback**: implemented. In open-loop mode it's a verification/stall-detection layer — it compares the encoder's physical reading against the step count and can halt into an error state on disagreement or disconnection. In closed-loop mode it's also the feedback signal that drives motion.
- **Closed-loop position control**: implemented, selectable at **compile time** via the `controlMode` global (`OPEN_LOOP` / `CLOSED_LOOP`) — flip it and reflash; there's no runtime toggle. A proportional servo drives the encoder onto the commanded setpoint (the jog soft-limit endpoints and the idle hold position). Open-loop is the baseline and is preserved unchanged.

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the state machine design and the encoder's calibration history, and [`CLAUDE.md`](CLAUDE.md) for repository-level notes aimed at AI coding assistants.
