from vex import *
import json
import sys
import uselect

brain = Brain()
controller_1 = Controller(PRIMARY)

# This layout was read back from the existing Brain-side X-Drive program.
front_right = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
front_left = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
rear_right = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
rear_left = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)

MAX_LINEAR_SPEED_MPS = 0.35
MAX_TURN_SPEED_DPS = 90.0
DEADBAND_PERCENT = 5
REMOTE_COMMAND_TIMEOUT_MS = 350
TELEMETRY_INTERVAL_MS = 50

stdin_poll = uselect.poll()
stdin_poll.register(sys.stdin, uselect.POLLIN)
stdin_buffer = ""
remote_motion = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
remote_deadline_ms = 0


def clamp(value, low, high):
    return max(low, min(high, value))


def apply_deadband(value):
    if abs(value) < DEADBAND_PERCENT:
        return 0
    return value


def controller_motion():
    forward_pct = apply_deadband(controller_1.axis2.position())
    strafe_pct = apply_deadband(controller_1.axis4.position())
    turn_pct = apply_deadband(controller_1.axis1.position())

    return {
        "x.vel": (forward_pct / 100.0) * MAX_LINEAR_SPEED_MPS,
        "y.vel": (strafe_pct / 100.0) * MAX_LINEAR_SPEED_MPS,
        "theta.vel": (turn_pct / 100.0) * MAX_TURN_SPEED_DPS,
    }


def motion_to_percent(motion):
    return (
        clamp((motion["x.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),
        clamp((motion["y.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),
        clamp((motion["theta.vel"] / MAX_TURN_SPEED_DPS) * 100.0, -100.0, 100.0),
    )


def apply_drive_motion(motion):
    forward_pct, strafe_pct, turn_pct = motion_to_percent(motion)

    front_left_pct = forward_pct + turn_pct + strafe_pct
    rear_left_pct = forward_pct - turn_pct + strafe_pct
    front_right_pct = forward_pct - turn_pct - strafe_pct
    rear_right_pct = forward_pct + turn_pct - strafe_pct

    front_left.set_velocity(front_left_pct, PERCENT)
    rear_left.set_velocity(rear_left_pct, PERCENT)
    front_right.set_velocity(front_right_pct, PERCENT)
    rear_right.set_velocity(rear_right_pct, PERCENT)

    front_left.spin(FORWARD)
    rear_left.spin(FORWARD)
    front_right.spin(FORWARD)
    rear_right.spin(FORWARD)


def read_motor_positions():
    return {
        "vex_front_right.pos": front_right.position(DEGREES),
        "vex_front_left.pos": front_left.position(DEGREES),
        "vex_rear_right.pos": rear_right.position(DEGREES),
        "vex_rear_left.pos": rear_left.position(DEGREES),
    }


def parse_numeric(payload, key, fallback=0.0):
    value = payload.get(key, fallback)
    if isinstance(value, (int, float)):
        return float(value)
    return float(fallback)


def handle_command_line(line, now_ms):
    global remote_deadline_ms

    try:
        payload = json.loads(line)
    except Exception:
        return

    if not isinstance(payload, dict):
        return

    command = payload.get("command")
    if command == "release":
        remote_deadline_ms = 0
        return

    if command not in (None, "velocity"):
        return

    remote_motion["x.vel"] = parse_numeric(payload, "x.vel")
    remote_motion["y.vel"] = parse_numeric(payload, "y.vel")
    remote_motion["theta.vel"] = parse_numeric(payload, "theta.vel")

    ttl_ms = payload.get("ttl_ms", REMOTE_COMMAND_TIMEOUT_MS)
    if not isinstance(ttl_ms, (int, float)):
        ttl_ms = REMOTE_COMMAND_TIMEOUT_MS
    remote_deadline_ms = now_ms + max(50, int(ttl_ms))


def poll_serial_commands(now_ms):
    global stdin_buffer

    while stdin_poll.poll(0):
        chunk = sys.stdin.read(1)
        if chunk is None:
            break
        if isinstance(chunk, bytes):
            chunk = chunk.decode("utf-8", "ignore")
        stdin_buffer += chunk

        while "\n" in stdin_buffer:
            line, stdin_buffer = stdin_buffer.split("\n", 1)
            if line:
                handle_command_line(line, now_ms)


def serial_motion_active(now_ms):
    return remote_deadline_ms > now_ms


def print_motion(motion, source):
    positions = read_motor_positions()
    print(
        '{"x.vel":%.4f,"y.vel":%.4f,"theta.vel":%.4f,"vex_front_right.pos":%.4f,"vex_front_left.pos":%.4f,"vex_rear_right.pos":%.4f,"vex_rear_left.pos":%.4f,"source":"%s"}'
        % (
            motion["x.vel"],
            motion["y.vel"],
            motion["theta.vel"],
            positions["vex_front_right.pos"],
            positions["vex_front_left.pos"],
            positions["vex_rear_right.pos"],
            positions["vex_rear_left.pos"],
            source,
        )
    )


def main():
    last_telemetry_ms = -TELEMETRY_INTERVAL_MS

    wait(30, MSEC)
    print("\033[2J")

    while True:
        now_ms = brain.timer.system()
        poll_serial_commands(now_ms)

        if serial_motion_active(now_ms):
            active_motion = dict(remote_motion)
            source = "pi"
        else:
            active_motion = controller_motion()
            source = "controller"

        apply_drive_motion(active_motion)

        if now_ms - last_telemetry_ms >= TELEMETRY_INTERVAL_MS:
            print_motion(active_motion, source)
            last_telemetry_ms = now_ms

        wait(20, MSEC)


main()
