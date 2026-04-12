#!/usr/bin/env bash
set -euo pipefail

source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate lerobot
cd "$HOME/lerobot"

if [[ -z "${LEKIWI_ROBOT_PORT:-}" ]]; then
  for candidate in /dev/serial/by-id/*USB_Single_Serial* /dev/ttyACM* /dev/ttyUSB*; do
    if [[ -e "$candidate" ]]; then
      LEKIWI_ROBOT_PORT="$candidate"
      break
    fi
  done
fi

exec python scripts/lekiwi_sts_power_host.py \
  --robot-id follow-mobile \
  --robot-port "${LEKIWI_ROBOT_PORT:-/dev/ttyACM0}" \
  --disable-cameras \
  --connection-time-s 3600 \
  --base-max-raw-velocity 6000 \
  --base-wheel-torque-limit 700 \
  "$@"
