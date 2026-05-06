# Robot Arm V2

Robot Arm V2 is a competition-focused control stack for an SO-101/LeKiwi-style
follower arm, an optional SO leader arm, a Raspberry Pi robot host, and a VEX V5
mobile base. It turns this repository into a local Mac dashboard that can start
live control, switch arm/base authority, record trajectories, replay saved
moves, hold arm poses, build chained routines, and capture training data.

The original SO-100/SO-101 hardware files are still in this repository. This
README is the practical guide for taking the repo, setting up your own robot,
and getting the dashboard working.

## What This Repository Contains

- A React dashboard in `src/App.tsx`.
- An Express backend in `server/index.ts`.
- Robot orchestration in `server/robotController.ts`.
- Pi-side helpers in `scripts/`, uploaded automatically to the Pi.
- SO-101 STL/STEP hardware files in `STL/`, `STEP/`, `Optional/`, and `media/`.
- Competition workflow documentation in `ROBOT_COMPETITION_CONTROL_SPEC.md`.
- Recovery and fallback notes in `LEKIWI_FALLBACK.md` and
  `LEKIWI_RECOVERY_LOG_2026-03-27.md`.
- Optional power and compliance diagrams in `output/diagrams/`.

## System Overview

The control stack has four active pieces:

1. Mac dashboard:
   - Runs the web UI and backend.
   - Starts local keyboard/leader teleop helpers.
   - Opens SSH/SFTP connections to the Pi.
   - Stores local dashboard settings in `.lekiwi-ui/config.json`.

2. Raspberry Pi:
   - Runs LeRobot and connects to the follower arm motor bus.
   - Runs `scripts/lekiwi_host.py` for live arm control and warm replay.
   - Stores recordings in `/home/pi/lekiwi-trajectories`.
   - Talks to the VEX Brain over USB serial when the base bridge is enabled.

3. SO follower arm:
   - The powered arm on the robot.
   - Receives commands from leader arm, keyboard arm control, replay, home, or
     hold pose logic.

4. VEX V5 base:
   - Can be driven locally by the VEX handheld controller.
   - Can also accept Pi-side keyboard/replay/preposition commands when enabled.
   - For a crowded competition arena, VEX controller base control is preferred
     because it avoids Mac-to-Wi-Fi-to-Pi base latency.

## Hardware You Need

Minimum for arm-only use:

- Mac computer.
- Raspberry Pi with Wi-Fi or Ethernet.
- SO-101 follower arm.
- Feetech/Waveshare motor controller board.
- STS3215 follower servos.
- Correct power supply for your servo chain.
- USB cable from Pi to follower motor controller.

Recommended for competition teleop:

- SO leader arm connected to the Mac over USB.
- VEX V5 Brain.
- VEX V5 Controller.
- VEX omni or mecanum base.
- USB from Pi to VEX Brain.
- VEX inertial sensor.
- Optional ultrasonic sensors for base prepositioning.

Useful hardware docs already in this repo:

- `SO101_BUILD_RUNBOOK.md` for print, assembly, and LeRobot setup notes.
- `SO101_BOM_Leader_Follower_Other_ALL.md` for leader/follower parts.
- `SO101_BOM_Milton_ON_CA.md` for a localized BOM.
- `3DPRINT.md` for print service notes.
- `Optional/VEX_V5_Base_Brain/README.md` for VEX base notes.
- `output/diagrams/robot-power-compliance-wiring.svg` for wiring guidance.
- `output/diagrams/robot-power-inspection-summary.svg` for inspection notes.

## Important Safety Notes

This project can move real hardware. Treat it like robot control software, not
a demo app.

- Keep a physical power switch or battery disconnect within reach.
- Start with the arm clear of people and objects.
- Use conservative torque limits until every joint is verified.
- Do not mix 7.4 V and 12 V servos on the same powered chain.
- If a servo overheats or disappears from the bus, stop and inspect the
  mechanical joint before retrying.
- If the dashboard safety latch trips, inspect the robot before restarting live
  control.
- Do not rely on Wi-Fi for emergency stop. Use physical power removal for
  immediate safety.

The runtime safety layer includes torque caps, temperature/stall checks,
gripper force limiting, and strict latching behavior. Those checks are meant to
interrupt convenience when the robot looks unsafe.

## Default Network And Paths

The dashboard defaults are defined in `server/defaultConfig.ts`.

| Setting | Default |
| --- | --- |
| Hotspot SSID | `rawr-hotspot` |
| Pi hostname | `rawr.local` |
| Pi fallback IP | `10.42.0.1` |
| Pi user | `pi` |
| Pi password | `password` |
| Pi LeRobot checkout | `/home/pi/lerobot` |
| Pi conda script | `/home/pi/miniforge3/etc/profile.d/conda.sh` |
| Pi helper directory | `/home/pi/.lekiwi-ui` |
| Pi recording directory | `/home/pi/lekiwi-trajectories` |
| Mac LeRobot checkout | `/Users/meharkhanna/lerobot` |
| Mac conda script | `/Users/meharkhanna/miniforge3/etc/profile.d/conda.sh` |
| Follower robot ID | `follow-mobile` |
| Follower robot port | `/dev/ttyACM0` |

Change these from the dashboard Settings panel for your own machine. The local
settings file is intentionally ignored by git because it can contain the Pi
password.

## VEX Base Defaults

| VEX setting | Default |
| --- | --- |
| Telemetry program slot | `8` |
| Replay program slot | `7` |
| Telemetry program name | `Base Telemetry` |
| Inertial sensor port | `4` |
| Front right motor | port `1`, reversed |
| Front left motor | port `2` |
| Rear right motor | port `9`, reversed |
| Rear left motor | port `10` |
| Forward/back axis | `axis3` |
| Strafe axis | `axis4` |
| Turn axis | `axis1` |
| Deadband | `5%` |
| Max linear speed | `0.35 m/s` |
| Max turn speed | `90 deg/s` |

The dashboard has a `Keyboard base listener` toggle:

- Off: VEX controller owns the base. This is the recommended competition mode.
- On: Mac keyboard commands are forwarded through Wi-Fi and the Pi to the VEX
  base.

When the toggle is off, the Pi host releases VEX base control and the Mac
keyboard teleop filters out base keys. This prevents the old conflict where the
keyboard path could send hold/zero commands while the controller tried to move.

## Install The Dashboard

Install Node.js 20 or newer, then from the repo root:

```bash
npm install
npm run dev
```

Open:

```text
http://localhost:5173
```

For a production-style local run:

```bash
npm run build
npm start
```

Open:

```text
http://localhost:4318
```

Useful scripts:

```bash
npm run dev
npm run build
npm run build:client
npm run build:server
npm run test:server
```

## Install LeRobot On The Mac

The Mac runs leader-arm calibration, local teleop, optional local replay, and
training.

Recommended path on macOS:

```bash
cd /tmp
curl -L -o Miniforge3-Darwin-arm64.sh \
  https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Darwin-arm64.sh
bash Miniforge3-Darwin-arm64.sh
```

Open a new shell, then:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda create -y -n lerobot python=3.12
conda activate lerobot
conda install -y ffmpeg=7.1.1 -c conda-forge
mkdir -p ~/src
cd ~/src
git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e ".[lekiwi]"
```

If your checkout is somewhere other than `/Users/meharkhanna/lerobot`, update
the Mac project path in Settings.

## Install LeRobot On The Pi

On the Pi:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda create -y -n lerobot python=3.12
conda activate lerobot
conda install -y ffmpeg=7.1.1 -c conda-forge
cd ~
git clone https://github.com/huggingface/lerobot.git
cd ~/lerobot
pip install -e ".[lekiwi]"
```

Make sure SSH is enabled and the Mac can reach the Pi:

```bash
ssh pi@rawr.local
ssh pi@10.42.0.1
```

The dashboard uploads helper scripts to `/home/pi/.lekiwi-ui` automatically, so
you normally do not need to copy files by hand.

## Prepare The SO-101 Arms

1. Print and assemble the follower arm.
2. Optional: print and assemble the leader arm.
3. Wire servos from base to gripper in logical order.
4. Power the servo bus with the correct voltage/current supply.
5. Connect the follower motor controller to the Pi.
6. Connect the leader motor controller to the Mac.
7. Run LeRobot port discovery:

```bash
conda activate lerobot
lerobot-find-port
```

8. Calibrate the follower on the Pi and the leader on the Mac using the current
   LeRobot calibration commands for your installed version.

The dashboard can detect a different leader serial port and use it instead of
the old hardcoded port, but calibration must still exist for the leader ID.

## Prepare The VEX Base

1. Build the VEX base with the motor ports listed above, or update Settings.
2. Connect the VEX Brain to the Pi over USB.
3. Put the inertial sensor on port `4`, or update Settings.
4. Use the dashboard `Sync VEX Telemetry` action to upload/sync the telemetry
   program.
5. Confirm the dashboard reports VEX telemetry as active before relying on
   base replay or prepositioning.

For competition driving, keep `Keyboard base listener` off and use the VEX
controller for base motion.

## Start Live Control

1. Join the Mac to the robot network, normally `rawr-hotspot`.
2. Start the dashboard:

```bash
npm run dev
```

3. Open `http://localhost:5173`.
4. Check Settings:
   - Pi host and fallback host.
   - Pi username/password.
   - Mac and Pi LeRobot paths.
   - Follower robot port.
   - VEX ports and reversals.
5. Choose arm source:
   - Leader arm.
   - Keyboard arm.
   - None.
6. Choose base source:
   - VEX controller.
   - Keyboard base listener.
7. Click `Start Control`.

The backend will:

- Reset old Pi SSH/SFTP sessions.
- Probe `rawr.local` and `10.42.0.1`.
- Detect the leader arm port.
- Upload helper scripts if changed.
- Start the Pi host over SSH.
- Start the Mac-side keyboard/leader teleop helper.
- Wait for host ports before reporting live control ready.

## Keyboard Controls

Arm keyboard controls:

| Keys | Joint |
| --- | --- |
| `Q` / `A` | shoulder pan |
| `W` / `S` | shoulder lift |
| `E` / `D` | elbow flex |
| `R` / `F`, `Y` / `H` | wrist flex |
| `T` / `G` | wrist roll |
| `Z` / `X`, `C` | gripper |

Base keyboard controls, only when `Keyboard base listener` is on:

| Keys | Base motion |
| --- | --- |
| Arrow up/down or `I`/`K` | forward/back |
| Arrow left/right or `J`/`L` | strafe |
| `O` / `P`, `U`, Enter hold modes | turn |
| `0` | toggle drive/ECU speed |

VEX Pin 5 servo helper keys:

| Key | Servo position |
| --- | --- |
| `N` | start |
| `B` | up |
| `V` | down |

## Recording And Replay

Recordings are JSON trajectory files on the Pi:

```text
/home/pi/lekiwi-trajectories
```

Dashboard recording modes include:

- Leader arm recording.
- Keyboard recording.
- Free-teach / hand-guide style capture.
- Pro Recording beta flows.

Replay options include:

- Target Pi follower arm.
- Target Mac-connected leader arm.
- Replay speed.
- Hold final pose duration.
- Include or exclude VEX base motion.
- Optional VEX start prepositioning.
- Home mode: none, start, end, or both.

Large recording details are downloaded over SFTP before parsing locally. This
avoids truncated SSH stdout JSON failures such as:

```text
Unterminated string in JSON at position 32768
```

If that error still appears after this change, the recording file itself is
likely partial or corrupt. Delete it or record again.

## Pinned Moves, Holds, And Chain-links

Pinned moves let you turn a recording into a dashboard button and optional
hotkey. They are useful for repeated competition actions.

A hold pin is a pinned recording whose final arm pose becomes the active hold.
While an arm hold is active:

- The arm stays at the hold pose.
- The VEX controller can still drive the base.
- Other arm-only Pi replays can return to the held pose afterwards.
- Keyboard arm override can temporarily take over and then return to the hold.

Chain-links are ordered sequences of recordings. They can run straight through
or ask for confirmation between steps.

## Training Workflows

The Training panel supports ACT-style local workflows:

- Capture datasets on the Pi.
- Capture leader-as-follower datasets on the Mac.
- Sync dataset metadata.
- Run local training.
- Deploy checkpoints to the Pi.
- Benchmark deployed policies.
- Run policy eval.

Default training paths:

| Path | Default |
| --- | --- |
| Pi dataset | `/home/pi/lerobot-datasets/<task>` |
| Mac dataset | `output/training-datasets/<task>` |
| Mac training run | `output/training-runs/<task>` |
| Pi deployed policy | `/home/pi/lerobot-training/<task>/deployed-policy` |
| Pi eval dataset | `/home/pi/lerobot-eval/<task>` |

The default profile is `Wood Pick`. Update the task text and paths in the
Training panel before collecting serious data.

## Low-Latency Competition Notes

Arena Wi-Fi can be slow because many robots share RF space. The code now does
these things to reduce perceived latency:

- Background SSH polling is skipped while the live Pi host owns the robot.
- Live ZMQ sockets are configured with latest-message behavior.
- The Mac teleop helper sends commands nonblocking.
- If Wi-Fi stalls, stale commands are dropped instead of queued.
- The Pi host drains queued live commands and executes only the newest command.
- Recording timeline JSON is fetched by SFTP instead of large SSH stdout.

Still, software cannot remove RF congestion. Best competition setup:

- Use VEX controller base control when possible.
- Keep `Keyboard base listener` off unless keyboard base control is required.
- Keep the Mac physically close to the robot access point.
- Avoid streaming cameras during critical control if you do not need them.
- Prefer Ethernet for setup, debugging, sync, and training transfers.
- Do not run training, dataset sync, or large downloads while live driving.

## Emergency And Recovery

Use the dashboard `Emergency Stop + Torque Off` first for normal software
shutdown.

If a Pi process is stuck, reset connections in the dashboard or SSH to the Pi
and kill LeKiwi helper processes. See:

```text
LEKIWI_FALLBACK.md
LEKIWI_RECOVERY_LOG_2026-03-27.md
```

Gripper overheat recovery is documented in `LEKIWI_FALLBACK.md`. Treat gripper
overheat as a mechanical binding or hard-stop issue until proven otherwise.

## Troubleshooting

Pi is not reachable:

- Confirm the Mac is on `rawr-hotspot`.
- Try `ssh pi@10.42.0.1`.
- Try `ssh pi@rawr.local`.
- Check Settings for username/password/host.
- Use Ethernet if Wi-Fi was disabled or overloaded.

Leader arm port mismatch:

- Run `lerobot-find-port`.
- Recalibrate the leader using the detected port.
- The dashboard can use a detected port at runtime, but calibration must exist.

VEX controller and keyboard base conflict:

- Turn `Keyboard base listener` off.
- Start live control again.
- The command line should include `--release-vex-controller-base true`.
- It should not include `--vex-live-base-control true`.

Recording JSON parse error at 32768 or 196608:

- This used to happen when large JSON came back truncated over SSH stdout.
- The current code downloads recording detail by SFTP.
- If the error remains, the recording file is probably incomplete. Re-record.

No command available:

- This can be normal while the host is waiting for live commands.
- If motion is delayed, check Wi-Fi quality and whether command drops are
  logged.

Servo safety latch:

- Stop and inspect the arm.
- Check temperature, wiring, power, and mechanical binding.
- Restart live control only after the cause is clear.

## Development Notes

Source layout:

| Path | Purpose |
| --- | --- |
| `src/App.tsx` | React dashboard |
| `src/styles.css` | Dashboard styles |
| `src/types.ts` | Client-side shared types |
| `server/index.ts` | Express API routes |
| `server/robotController.ts` | Main orchestration logic |
| `server/defaultConfig.ts` | Default settings |
| `server/configStore.ts` | Local config persistence |
| `scripts/lekiwi_host.py` | Pi live host and warm replay |
| `scripts/lekiwi_keyboard_teleop.py` | Mac keyboard/leader live teleop |
| `scripts/lekiwi_record_trajectory.py` | Pi recording helper |
| `scripts/lekiwi_replay_trajectory.py` | Pi replay helper |
| `scripts/lekiwi_runtime.py` | Safety, torque, sensor, runtime utilities |
| `scripts/vex_base_bridge.py` | VEX Brain bridge and VEX program generation |
| `tests/` | Python helper tests |
| `server/*.test.ts` | Backend tests |

Run verification before pushing:

```bash
python3 -m py_compile scripts/lekiwi_keyboard_teleop.py scripts/lekiwi_host.py
python3 -m unittest tests/test_keyboard_teleop.py
npm run build:client
npm run build:server
npm run test:server
```

Do not commit these local artifacts:

- `.lekiwi-ui/config.json`
- `.tmp/`
- `scripts/__pycache__/`
- `.playwright-mcp/*.log`
- `dist/`
- `node_modules/`

## Making This Work On Your Own Robot

Use this checklist from a fresh clone:

1. Build or buy the SO-101 follower arm.
2. Optional: build the SO leader arm.
3. Build the VEX base and wire it to the default ports, or update Settings.
4. Install LeRobot on the Mac and Pi.
5. Confirm SSH from Mac to Pi.
6. Confirm follower motor controller appears on the Pi.
7. Confirm leader motor controller appears on the Mac.
8. Calibrate follower and leader.
9. Start the dashboard with `npm run dev`.
10. Update Settings for your paths, ports, Pi credentials, and VEX layout.
11. Run `Sync VEX Telemetry` if using the VEX base.
12. Start live control with VEX controller base mode first.
13. Test one joint at a time at low torque.
14. Test base driving from the VEX controller.
15. Record a short trajectory.
16. Replay it with base motion disabled.
17. Add pinned moves and holds only after the basic path is reliable.

## Related Documents

- `ROBOT_COMPETITION_CONTROL_SPEC.md`: source of truth for authority and
  contest behavior.
- `CONTROL_UI.md`: older UI behavior notes.
- `SO101_BUILD_RUNBOOK.md`: SO-101 build and LeRobot setup notes.
- `LEKIWI_FALLBACK.md`: fallback hosts and recovery commands.
- `LEKIWI_RECOVERY_LOG_2026-03-27.md`: debugging history and known-good
  recovery steps.
- `LEKIWI_WOODEN_PICK_AUTONOMY.md`: wood pick autonomy notes.
- `CHAINLINK_VALIDATE_TEST_REPORT.md`: Chain-link validation report.

## License

This repository inherits the upstream SO-ARM100/SO-101 license file included in
`LICENSE`. Check third-party dependencies and VEX/LeRobot licensing before
redistributing a bundled robot image.
