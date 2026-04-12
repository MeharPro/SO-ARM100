#!/bin/bash
set -euo pipefail

source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate lerobot
cd "$HOME/lerobot"

pkill -f 'python -m lerobot.robots.lekiwi.lekiwi_smooth_host' >/dev/null 2>&1 || true
pkill -f 'python -m lerobot.robots.lekiwi.lekiwi_host' >/dev/null 2>&1 || true
sleep 1

exec python -m lerobot.robots.lekiwi.lekiwi_smooth_host \
  --robot-id=follow-mobile \
  --robot-port=/dev/ttyACM0 \
  --robot-max-relative-target=5 \
  --loop-hz=25 \
  --arm-p-coefficient=24 \
  --arm-d-coefficient=16 \
  --arm-alpha=0.85 \
  --gripper-alpha=0.80 \
  --base-alpha=0.95 \
  --arm-deadband=0.02 \
  --gripper-deadband=0.02 \
  --shoulder-max-step=12.0 \
  --elbow-max-step=12.0 \
  --wrist-max-step=14.0 \
  --wrist-roll-max-step=18.0 \
  --gripper-max-step=4.0 \
  --connection-time-s=3600 \
  "$@"
