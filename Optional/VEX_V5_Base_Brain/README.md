# VEX V5 Base Telemetry Bridge

This folder contains a VEXcode V5 Python template that keeps manual controller drive on the V5 Brain while streaming base motion telemetry plus wheel encoder positions to the Raspberry Pi over the Brain's USB console serial port.

Use this when:

- the V5 controller should remain the live/manual backup
- the Pi should record base motion through the existing robot-arm UI
- the Pi should replay recorded base motion through the existing robot-arm UI
- the old LeKiwi base servos must stay disabled

The Mac UI now owns the normal slot-8 telemetry program. Edit the VEX controls in
the UI settings page, then use `Save + Run VEX Telemetry` to push the current
mapping to the Brain.

## Pi-side expectations

The Pi helper auto-detects the V5 Brain USB serial device and expects newline-delimited JSON with these keys:

```json
{
  "x.vel": 0.12,
  "y.vel": -0.03,
  "theta.vel": 18.0,
  "vex_front_right.pos": 42.5,
  "vex_front_left.pos": -18.0,
  "vex_rear_right.pos": 39.2,
  "vex_rear_left.pos": -21.3
}
```

- `x.vel` and `y.vel` are meters per second
- `theta.vel` is degrees per second
- `vex_*.pos` values are V5 motor encoder positions in degrees
- `source` is either `controller` or `pi`

For normal use, keep `Base Telemetry` running on the Brain. That slot preserves
manual controller drive and keeps streaming telemetry to the Pi.

For base-inclusive replay, the Pi now uses the official `vexcom` tool to upload a
temporary replay program into a separate Brain slot and runs that slot for the
duration of the replay. That replay slot first drives the four V5 motors back to
the wheel encoder positions captured at the start of recording, then starts the
timed replay. When replay ends, the Pi restores the normal slot-8
telemetry/controller program automatically.

## Current hardware layout

The current file is based on the existing Brain-side `X-Drive` program that was
read back from the Brain through the official `vexcom` tool. It uses:

- `PORT1` front right, reversed
- `PORT2` front left
- `PORT9` rear right, reversed
- `PORT10` rear left
- `axis3` forward/back
- `axis4` strafe
- `axis1` turn

Only change the scaling constants if you want different recorded velocity magnitudes.

## Brain slots

- slot 8: `Base Telemetry` for normal controller driving plus telemetry streaming
- slot 7: temporary `Base Replay` program generated and launched by the Pi

## USB ports on the Pi

On your Pi, the V5 Brain currently appears as:

- `/dev/serial/by-id/...VEX_Robotics_V5_Brain...-if00`
- `/dev/serial/by-id/...VEX_Robotics_V5_Brain...-if02`

The helper prefers the `if02` console serial interface first.

## Safety

- This template is meant to preserve the current manual VEX controller driving.
- For normal use, run slot 8 and drive manually with the VEX controller.
- During replay, the Pi temporarily runs the generated slot-7 replay program.
- When replay finishes or is stopped, the Pi restores slot 8 automatically.
- The robot-arm UI hosts are configured to keep the old LeKiwi base disabled.
- Only the arm path should be actuated by the Pi.
