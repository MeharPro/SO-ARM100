#!/bin/bash
set -euo pipefail

source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd ~/lerobot
exec python -m lerobot.robots.lekiwi.lekiwi_host --host.connection_time_s=3600 --robot.id=follow-mobile
