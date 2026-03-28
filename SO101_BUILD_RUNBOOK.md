# SO-101 Build Runbook

Last updated: 2026-03-25
Workspace: `/Users/meharkhanna/robot-arm`
Host: macOS arm64

This runbook ties together:

- the hardware files already in this repo
- the current Hugging Face SO-101 assembly and calibration docs
- the current LeRobot installation requirements

## 1. Pick Your Hardware Path

Choose one path before wiring anything:

- `Follower only`
  - Build and calibrate one follower arm.
  - Useful if you already have a dataset, plan to use replay/autonomy, or will use another teleop device later.
- `Leader + follower`
  - This is the standard SO-101 teleoperation setup from the official docs.
- `12V follower upgrade`
  - Your local BOM in [`SO101_BOM_Milton_ON_CA.md`](./SO101_BOM_Milton_ON_CA.md) uses a 12V follower.
  - In that setup, the follower chain must be all 12V STS3215 motors with its own 12V 5A+ supply.
  - Never mix 7.4V and 12V motors on the same powered daisy-chain.

## 2. Print The Right Files

This repo already contains the production print files.

If your bed is about `220 x 220 mm`:

- Follower: [`STL/SO101/Follower/Ender_Follower_SO101.stl`](./STL/SO101/Follower/Ender_Follower_SO101.stl)
- Leader: [`STL/SO101/Leader/Ender_Leader_SO101.stl`](./STL/SO101/Leader/Ender_Leader_SO101.stl)

If your bed is about `205 x 250 mm`:

- Follower: [`STL/SO101/Follower/Prusa_Follower_SO101.stl`](./STL/SO101/Follower/Prusa_Follower_SO101.stl)
- Leader: [`STL/SO101/Leader/Prusa_Leader_SO101.stl`](./STL/SO101/Leader/Prusa_Leader_SO101.stl)

Useful references in this repo:

- Print guidance: [`README.md`](./README.md)
- External print service notes: [`3DPRINT.md`](./3DPRINT.md)
- Full CAD assembly: [`STEP/SO101/SO101 Assembly.step`](./STEP/SO101/SO101%20Assembly.step)
- Individual part files: [`STL/SO101/Individual`](./STL/SO101/Individual)

Recommended print settings from the repo README:

- PLA+
- `0.4 mm` nozzle at `0.2 mm` layers, or `0.6 mm` nozzle at `0.4 mm`
- `15%` infill
- supports everywhere, but avoid clogging horizontal screw holes

Before committing to a full print, use the gauges in [`STL/Gauges`](./STL/Gauges).

## 3. Hardware Prep Before Assembly

Do this before touching software:

- Remove all support material cleanly.
- Separate `M2x6`, `M3x6`, and horn screws into different bins.
- Keep leader-only and follower-only wrist/gripper parts separate.
- Dry fit the printed parts with the servo gauge if tolerances look tight.

Leader-specific printed parts live here:

- [`STL/SO101/Individual/Handle_SO101.stl`](./STL/SO101/Individual/Handle_SO101.stl)
- [`STL/SO101/Individual/Trigger_SO101.stl`](./STL/SO101/Individual/Trigger_SO101.stl)
- [`STL/SO101/Individual/Wrist_Roll_SO101.stl`](./STL/SO101/Individual/Wrist_Roll_SO101.stl)

Follower-specific printed parts live here:

- [`STL/SO101/Individual/Moving_Jaw_SO101.stl`](./STL/SO101/Individual/Moving_Jaw_SO101.stl)
- [`STL/SO101/Individual/Wrist_Roll_Follower_SO101.stl`](./STL/SO101/Individual/Wrist_Roll_Follower_SO101.stl)

## 4. Servo Layout You Need To Get Right

Follower arm:

- `6x` STS3215 motors with `1/345` gearing
- If you follow your Milton BOM, that follower is the 12V variant

Leader arm:

| Joint | Name | Ratio |
| --- | --- | --- |
| 1 | Base / shoulder pan | `1/191` |
| 2 | Shoulder lift | `1/345` |
| 3 | Elbow flex | `1/191` |
| 4 | Wrist flex | `1/147` |
| 5 | Wrist roll | `1/147` |
| 6 | Gripper | `1/147` |

Plan for logical bus IDs `1 -> 6` from base to end effector. After setup, the controller should connect to motor `1` first, then continue down the daisy-chain.

## 5. Assembly Order

Use the official SO-101 page for the full visual guide. The high-level order is:

1. Joint 1: base motor, holder, horns, shoulder shell
2. Joint 2: shoulder lift motor, horns, upper arm
3. Joint 3: elbow motor, horns, forearm
4. Joint 4: wrist flex holder and motor
5. Joint 5: wrist roll motor and wrist structure
6. End effector:
   - follower: gripper hardware
   - leader: handle / trigger assembly

Two practical assembly notes from the official docs:

- Clean all support material before starting.
- It helps to place a 3-pin cable in each motor while access is easy.

## 6. Software Bring-Up On This Mac

Current machine state:

- `python3` is `3.9.6`
- `conda` is not on `PATH`
- `uv` is not on `PATH`
- system `ffmpeg` is `8.0.1`

Why this matters:

- the current LeRobot install docs call for Python `>= 3.12`
- the current docs explicitly say `ffmpeg 8.x` is not yet supported

Recommended path: install Miniforge, create a fresh `lerobot` env, and use `ffmpeg 7.1.1` inside that env.

### Install Miniforge

If you want the least ambiguous path on macOS arm64:

```bash
cd /tmp
curl -L -o Miniforge3-Darwin-arm64.sh \
  https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Darwin-arm64.sh
bash Miniforge3-Darwin-arm64.sh
```

Then open a new shell, or manually activate conda if needed:

```bash
source ~/miniforge3/bin/activate
```

### Create The LeRobot Environment

```bash
conda create -y -n lerobot python=3.12
conda activate lerobot
conda install -y ffmpeg=7.1.1 -c conda-forge
ffmpeg -version
```

Expected result: `ffmpeg 7.x`, not `8.x`.

### Install LeRobot With Feetech Support

Quick path using source install:

```bash
mkdir -p ~/src
cd ~/src
git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e ".[feetech]"
```

That single command installs LeRobot and the Feetech SDK support needed for SO-101 motors.

## 7. Find Your USB Ports

Connect one controller board by USB and power, then run:

```bash
conda activate lerobot
lerobot-find-port
```

On macOS, you should get paths like:

```text
/dev/tty.usbmodemXXXXXXXX
```

Do this once for the follower board and once for the leader board.

## 8. Program Motor IDs And Baudrate

Important rule:

- During `lerobot-setup-motors`, only connect one motor to the controller board at a time.
- Do not daisy-chain the full arm during EEPROM setup.

If you are using a Waveshare controller board, make sure the jumpers are on channel `B` for USB.

### Follower

```bash
conda activate lerobot
lerobot-setup-motors \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodemFOLLOWER_PORT
```

Follow the prompt order exactly. The script sets the IDs and baudrate for you.

### Leader

```bash
conda activate lerobot
lerobot-setup-motors \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodemLEADER_PORT
```

When both sides are done, reconnect the arm as a daisy-chain and plug the controller into motor `1`.

## 9. Calibrate Both Arms

Use stable, reusable IDs. Reuse the same IDs later for teleop, record, and replay.

### Follower

```bash
conda activate lerobot
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodemFOLLOWER_PORT \
  --robot.id=so101_follower_main
```

### Leader

```bash
conda activate lerobot
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodemLEADER_PORT \
  --teleop.id=so101_leader_main
```

During calibration:

- first move the arm to the midpoint of each joint range
- then sweep each joint through its full range as instructed

## 10. First Motion Test

If you have both a leader and follower:

```bash
conda activate lerobot
python -m lerobot.teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodemFOLLOWER_PORT \
  --robot.id=so101_follower_main \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodemLEADER_PORT \
  --teleop.id=so101_leader_main
```

This will also catch missing calibration and prompt for it if necessary.

If you only have a follower arm:

- stop after follower calibration and verify the joints respond cleanly
- for direct manual teleop you will still need a teleop device such as a leader arm or another supported controller path

## 11. First Dataset Smoke Test

Once teleop is stable, a minimal record command looks like this:

```bash
conda activate lerobot
python -m lerobot.record \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodemFOLLOWER_PORT \
  --robot.id=so101_follower_main \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodemLEADER_PORT \
  --teleop.id=so101_leader_main \
  --dataset.repo_id=YOUR_HF_USER/so101-smoke-test \
  --dataset.num_episodes=2 \
  --dataset.single_task="Pick up one object" \
  --dataset.push_to_hub=False
```

Start without cameras until the arm itself is reliable.

## 12. Common Gotchas

- `ffmpeg 8.x` is currently unsupported by the docs. Use `ffmpeg 7.x` inside the conda env.
- If you are using the 12V follower upgrade, keep the follower on its own 12V supply and never mix voltages in one chain.
- `lerobot-setup-motors` requires one motor at a time, not the whole arm plugged in.
- Use the same `robot.id` and `teleop.id` every time for a given physical setup.
- If a motor does not respond during setup, check power, USB, and the 3-pin servo cable before assuming a software problem.

## 13. Source Links

- Official SO-101 guide: <https://huggingface.co/docs/lerobot/en/so101>
- Official installation guide: <https://huggingface.co/docs/lerobot/en/installation>
- Getting started with real-world robots: <https://huggingface.co/docs/lerobot/main/getting_started_real_world_robot>
