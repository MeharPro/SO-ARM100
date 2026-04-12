#!/bin/zsh
set -euo pipefail

source /Users/meharkhanna/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd /Users/meharkhanna/lerobot
exec python examples/lekiwi/teleoperate.py
