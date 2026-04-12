#!/bin/bash
set -euo pipefail

source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate lerobot
cd "$HOME/lerobot"

pkill -f 'python -m lerobot.robots.lekiwi.lekiwi_smooth_host' >/dev/null 2>&1 || true
sleep 1

exec python -m lerobot.robots.lekiwi.lekiwi_smooth_host \
  --robot-id=follow-mobile \
  --robot-port=/dev/ttyACM0 \
  --disable-cameras \
  --robot-max-relative-target=5 \
  --loop-hz=10 \
  --arm-alpha=0.35 \
  --gripper-alpha=0.25 \
  --base-alpha=0.40 \
  --arm-deadband=0.35 \
  --gripper-deadband=0.15 \
  --shoulder-max-step=3.0 \
  --elbow-max-step=3.0 \
  --wrist-max-step=3.0 \
  --wrist-roll-max-step=4.0 \
  --gripper-max-step=1.0 \
  --connection-time-s=3600 \
  "$@"
