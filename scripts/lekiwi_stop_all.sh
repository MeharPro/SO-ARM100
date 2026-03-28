#!/bin/zsh
set -euo pipefail

pick_pi_host() {
  local candidates=()
  if [[ -n "${LEKIWI_PI_HOST:-}" ]]; then
    candidates+=("${LEKIWI_PI_HOST}")
  fi
  candidates+=("10.42.0.1" "rawr.local")

  local host
  for host in "${candidates[@]}"; do
    if ssh -o BatchMode=yes -o ConnectTimeout=3 "pi@${host}" "exit 0" >/dev/null 2>&1; then
      print -r -- "${host}"
      return 0
    fi
  done

  print -u2 "Could not reach the Pi on 10.42.0.1 or rawr.local. Set LEKIWI_PI_HOST if needed."
  return 1
}

PI_HOST="$(pick_pi_host)"
ROBOT_PORT="${LEKIWI_ROBOT_PORT:-/dev/ttyACM0}"

export LEKIWI_PI_HOST="${PI_HOST}"

ssh -qT "pi@${PI_HOST}" <<EOF
set -euo pipefail

if command -v lsof >/dev/null 2>&1; then
  pids="\$(lsof -tiTCP:5555 -sTCP:LISTEN || true) \$(lsof -tiTCP:5556 -sTCP:LISTEN || true)"
  if [ -n "\$pids" ]; then
    kill \$pids >/dev/null 2>&1 || true
    sleep 1
    kill -9 \$pids >/dev/null 2>&1 || true
  fi
fi
sleep 1

source "\$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate lerobot
cd "\$HOME/lerobot"

python - <<'PY'
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

motors = {
    f"m{i}": Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
    for i in range(1, 10)
}

bus = FeetechMotorsBus(
    port="${ROBOT_PORT}",
    motors=motors,
)

bus.connect(handshake=False)
disabled = []
failed = []
for motor_name in motors:
    try:
        bus.disable_torque(motor_name)
        disabled.append(motor_name)
    except Exception as exc:
        failed.append((motor_name, str(exc)))
bus.disconnect(disable_torque=False)
print(f"Disabled torque on: {', '.join(disabled) if disabled else 'none'}")
if failed:
    print("Failed to disable torque on:")
    for motor_name, error in failed:
        print(f"  - {motor_name}: {error}")
PY
EOF
