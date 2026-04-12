# LeKiwi Recovery Log

Last updated: 2026-03-27
Workspace: `/Users/meharkhanna/robot-arm`

This file records the LeKiwi install, network recovery, calibration, and servo-bus debugging work from March 27, 2026 so future me or future Codex can resume quickly.

## Current Known-Good State

As of the last verified run:

- Pi host ran cleanly with servo `6` included.
- Direct probe on the Pi saw all motors `1..9`.
- Individual reads on the Pi worked for all arm motors, including `arm_gripper`.
- Short `lekiwi_host` run on the Pi completed cleanly with `Cycle time reached.` and no motor-bus crash.

Known working host command on the Pi:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd ~/lerobot
python -m lerobot.robots.lekiwi.lekiwi_host --host.connection_time_s=3600 --robot.id=follow-mobile
```

Known working Mac command:

```zsh
source /Users/meharkhanna/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd /Users/meharkhanna/lerobot
python examples/lekiwi/teleoperate.py
```

Mac teleop assumptions:

- Pi IP: `192.168.40.135`
- Leader port: `/dev/tty.usbmodem5AE60840411`

## Machines And Paths

Mac:

- Clean LeRobot checkout: `/Users/meharkhanna/lerobot`
- Backup of older modified checkout: `/Users/meharkhanna/lerobot-backup-20260327-121838`
- Conda env: `/Users/meharkhanna/miniforge3/envs/lerobot`

Pi:

- Hostname: `rawr`
- Typical Ethernet IP during recovery: `192.168.40.135`
- LeRobot checkout: `/home/pi/lerobot`
- Conda env: `~/miniforge3/envs/lerobot`

## Install And Environment Work

What was done on the Mac:

- Old `~/lerobot` was backed up, not destroyed.
- Fresh upstream `huggingface/lerobot` was cloned into `/Users/meharkhanna/lerobot`.
- Fresh `lerobot` conda env was created with Python `3.12`.
- `ffmpeg 7.1.1` was installed because current LeRobot docs do not want `ffmpeg 8.x`.
- LeRobot extras were installed so LeKiwi could import `zmq`.

Important install detail:

- `pip install lerobot` alone was not enough for LeKiwi.
- The working install path for LeKiwi is:

```bash
pip install -e ".[lekiwi]"
```

## Network Recovery

At one point Wi-Fi on the Pi was accidentally disabled via the desktop menu.

Observed state after recovery by Ethernet:

- `eth0` connected
- `wlan0` unavailable
- Pi reachable by Ethernet at `192.168.40.135`

Takeaway:

- Ethernet came up independently of Wi-Fi.
- `.local` name resolution was unreliable during recovery.
- Direct SSH by IP was the dependable path.

## Leader Calibration

Working leader calibration command on the Mac:

```zsh
source /Users/meharkhanna/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd /Users/meharkhanna/lerobot

lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodem5AE60840411 \
  --teleop.id=leader
```

That saved calibration to:

- `/Users/meharkhanna/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/leader.json`

## Core Failure Pattern We Chased

Initial Pi-side failure looked like:

- `Failed to sync read 'Present_Position' on ids=[1, 2, 3, 4, 5, 6]`
- sometimes `Incorrect status packet`
- sometimes `There is no status packet`

Important distinction:

- `No command available` warnings from `lekiwi_host` are normal when the Mac client is not connected yet.
- The real blocker was motor-bus communication, not ZMQ.

## GitHub Issue Review

We reviewed LeRobot issue `#1252`:

- retrying `sync_read(..., num_retry=3)` helped some users
- some people needed larger retry counts
- later comments pointed to power, bus noise, USB hub issues, and bad physical states

Takeaway for this setup:

- retries were worth trying
- retries were not the root cause
- the bus problem was ultimately tied to the gripper servo branch

## Pi Code Changes During Debugging

The main file on the Pi that was edited repeatedly:

- `/home/pi/lerobot/src/lerobot/robots/lekiwi/lekiwi.py`

Changes tried during debugging:

- increased retry counts on `sync_read`
- increased retry counts in `configure()` writes and `enable_torque()`
- temporarily replaced arm `sync_read` with per-motor `read()`
- temporarily excluded servo `6` in software
- later restored servo `6` after the physical issue was resolved

Useful outcome:

- switching from grouped `sync_read` to per-motor `read()` exposed the problem more clearly
- once isolated, servo `6` or its branch was the dominant fault domain

## Probe Command That Was Most Useful

This was the most useful diagnostic on the Pi because it bypassed host handshake and showed the bus state directly:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd ~/lerobot

python - <<'PY'
from pprint import pprint
from lerobot.robots.lekiwi.lekiwi import LeKiwi
from lerobot.robots.lekiwi.config_lekiwi import LeKiwiConfig

robot = LeKiwi(LeKiwiConfig())
bus = robot.bus
bus.connect(handshake=False)

print("Broadcast ping:")
try:
    pprint(bus.broadcast_ping(num_retry=5, raise_on_error=True))
except Exception as e:
    print("broadcast error:", e)

print("\nIndividual arm reads:")
for motor in robot.arm_motors:
    try:
        value = bus.read("Present_Position", motor, num_retry=10)
        print(f"{motor}: OK -> {value}")
    except Exception as e:
        print(f"{motor}: ERROR -> {e}")

bus.disconnect(disable_torque=False)
PY
```

## What The Probe Revealed

Key observations from repeated tests:

1. There were times when all motors `1..9` were visible and all individual arm reads worked, but grouped `sync_read` still failed.
2. Excluding servo `6` in software and unplugging it physically let the host run cleanly.
3. When servo `6` was physically plugged in during the bad state, motor `1` could disappear too.
4. When servo `6` was physically unplugged, motor `1` came back immediately.

Strong causal conclusion from that phase:

- the servo `6` branch was capable of destabilizing the entire arm bus, including motor `1`

That did not prove whether the root cause was:

- the servo `6` unit itself
- the cable from `5 -> 6`
- connector seating/orientation
- the port on servo `5`
- or the replacement servo needing setup

But it did prove that attaching servo `6` could make the bus unhealthy.

## Replacement Gripper Servo Work

At one point the gripper servo was physically replaced.

Very plausible root cause after replacement:

- the new servo likely needed proper motor-controller setup so it had the correct ID and baudrate for `arm_gripper`

One failed attempt happened because the Pi software was temporarily in a `6`-excluded state, so:

```text
KeyError: 'arm_gripper'
```

This did not mean setup was impossible. It only meant the current `LeKiwiConfig()` instance no longer contained `arm_gripper`.

The right one-off setup pattern for only the gripper was:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd ~/lerobot

python - <<'PY'
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

bus = FeetechMotorsBus(
    port="/dev/ttyACM0",
    motors={
        "arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
    },
    calibration={},
)

bus.setup_motor("arm_gripper")
print("Configured arm_gripper as id", bus.motors["arm_gripper"].id)
PY
```

Inference:

- after the replacement and gripper-only motor setup path, servo `6` eventually came back and later verification passed with all `1..9` visible

## Final Successful Verification

The final successful verification with servo `6` included looked like this:

- `broadcast_ping` returned all motors:

```text
{1: 777, 2: 777, 3: 777, 4: 777, 5: 777, 6: 777, 7: 777, 8: 777, 9: 777}
```

- Individual arm reads succeeded for:
  - `arm_shoulder_pan`
  - `arm_shoulder_lift`
  - `arm_elbow_flex`
  - `arm_wrist_flex`
  - `arm_wrist_roll`
  - `arm_gripper`

- A short host run completed cleanly:

```text
Cycle time reached.
Shutting down Lekiwi Host.
```

That is the strongest known-good checkpoint from this recovery session.

## Ctrl+C Torque Safety Patch

We hardened the Pi host cleanup so a `Ctrl+C` does a best-effort stop and torque disable even if the normal disconnect path is interrupted or partially connected.

Patched file on the Pi:

- `/home/pi/lerobot/src/lerobot/robots/lekiwi/lekiwi_host.py`

What changed conceptually:

- added a `_shutdown_robot_safely(robot)` helper
- in `finally`, replaced the direct `robot.disconnect()` call with that helper
- the helper:
  - tries `robot.stop_base()`
  - tries `robot.bus.disconnect(robot.config.disable_torque_on_disconnect)`
  - if that fails, explicitly calls `robot.bus.disable_torque(num_retry=10)`
  - then closes the bus without trying to disable torque a second time
  - disconnects cameras best-effort

This was verified on the Pi by:

1. starting `lekiwi_host`
2. sending a real `Ctrl+C`
3. reconnecting to the bus and reading `Torque_Enable`

Verified post-`Ctrl+C` state:

```text
arm_shoulder_pan: Torque_Enable=0
arm_gripper: Torque_Enable=0
base_left_wheel: Torque_Enable=0
```

Interpretation:

- after the patch, `Ctrl+C` on the Pi host did actually release torque on the checked motors

## Meaning Of Negative Motor Values

Negative arm values seen in the probe were not the bug.

Those values were already decoded and normalized positions. Negative values are normal in that representation.

The real failures were packet-transport failures such as:

- `Incorrect status packet`
- `There is no status packet`

## What To Check First If It Breaks Again

If the bus fails again:

1. Run the direct probe above before changing code.
2. If motor `6` and motor `1` disappear together, suspect the gripper branch first.
3. Check:
   - servo `6`
   - cable from `5 -> 6`
   - connector seating on both ends
   - pass-through port on servo `5`
4. If the replacement servo was changed again, re-run the one-off gripper setup.
5. Only after that, retry `lekiwi_host`.

## Minimal Commands To Resume

Pi:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd ~/lerobot
python -m lerobot.robots.lekiwi.lekiwi_host --host.connection_time_s=3600 --robot.id=follow-mobile
```

Mac:

```zsh
source /Users/meharkhanna/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd /Users/meharkhanna/lerobot
python examples/lekiwi/teleoperate.py
```

## Record And Replay Exact Follower Motion

These trajectory tools are intended to run on the Pi.

The source files live locally at:
- `/Users/meharkhanna/robot-arm/scripts/lekiwi_record_trajectory.py`
- `/Users/meharkhanna/robot-arm/scripts/lekiwi_replay_trajectory.py`

Copy them onto the Pi first if the `robot-arm` repo is not already checked out there.

### Record

Run the recorder on the Pi instead of the normal host. It behaves like the host, accepts teleop from the Mac, and also records the follower's exact observed servo motion to disk.

Pi:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd ~/robot-arm
python scripts/lekiwi_record_trajectory.py \
  --robot.id=follow-mobile \
  --host.connection_time_s=3600 \
  --output ~/lekiwi-trajectories/demo.json
```

Mac:

```zsh
source /Users/meharkhanna/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd /Users/meharkhanna/lerobot
python examples/lekiwi/teleoperate.py
```

The saved file is the follower's observed state, not just the leader commands.

### Replay

Replay the exact recorded follower trajectory locally from the Pi:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd ~/robot-arm
python scripts/lekiwi_replay_trajectory.py \
  --robot.id=follow-mobile \
  --input ~/lekiwi-trajectories/demo.json
```

Useful flags:
- `--include-base` to replay the recorded base velocities too.
- `--speed 0.5` for half-speed replay.
- `--speed 2.0` for double-speed replay.

## Extra Notes

- The Pi-side `configure()` path was hardened during debugging with `num_retry=10` on torque/config writes.
- During recovery, Ethernet was the reliable way back into the Pi.
- If `.local` lookup breaks again, use the Pi IP directly.
- If host warnings only say `No command available`, that is expected before the Mac teleop client connects.
