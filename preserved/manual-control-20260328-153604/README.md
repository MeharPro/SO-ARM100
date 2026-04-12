# Preserved Manual Control Bundle

This snapshot preserves the currently working manual LeKiwi control path.

Do not modify the live repos used by these commands:

- `/Users/meharkhanna/lerobot`
- `/home/pi/lerobot`

Use these sandboxes for any future experimentation instead:

- `/Users/meharkhanna/sandboxes/lerobot-sandbox-20260328-153604`
- `/Users/meharkhanna/sandboxes/skills2026-lekiwi-20260328-153604`
- `/home/pi/sandboxes/lerobot-sandbox-20260328-153605`
- `/home/pi/sandboxes/skills2026-lekiwi-20260328-153605`

Live manual commands preserved here:

Mac:

```bash
source /Users/meharkhanna/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd /Users/meharkhanna/lerobot
python examples/lekiwi/teleoperate.py
```

Pi:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd ~/lerobot
python -m lerobot.robots.lekiwi.lekiwi_host --host.connection_time_s=3600 --robot.id=follow-mobile
```

Snapshot contents:

- `mac/teleoperate.py`: current manual teleop script
- `mac/lekiwi.py`: current local LeKiwi implementation
- `mac/lerobot_commit.txt`: current local `lerobot` commit
- `mac/lerobot_status.txt`: current local `lerobot` dirty state
- `mac/sha256.txt`: checksums for preserved local files
- `pi/lekiwi_host.py`: current Pi host implementation
- `pi/lekiwi.py`: current Pi LeKiwi implementation
- `pi/config_lekiwi.py`: current Pi LeKiwi config
- `pi/lerobot_commit.txt`: current Pi `lerobot` commit
- `pi/lerobot_status.txt`: current Pi `lerobot` dirty state
- `pi/sha256.txt`: checksums for preserved Pi files

Run helpers:

- `mac/run_manual_teleop.sh`
- `pi/run_manual_host.sh`
