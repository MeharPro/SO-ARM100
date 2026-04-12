# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field
from pathlib import Path

from lerobot.cameras.configs import CameraConfig, Cv2Rotation
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

from ..config import RobotConfig


def lekiwi_cameras_config() -> dict[str, CameraConfig]:
    return {
        "front": OpenCVCameraConfig(
            index_or_path="/dev/video0", fps=30, width=640, height=480, rotation=Cv2Rotation.ROTATE_180
        ),
        "wrist": OpenCVCameraConfig(
            index_or_path="/dev/video2", fps=30, width=480, height=640, rotation=Cv2Rotation.ROTATE_90
        ),
    }


def default_lekiwi_port() -> str:
    by_id_dir = Path("/dev/serial/by-id")
    if by_id_dir.exists():
        for pattern in ("usb-1a86_USB_Single_Serial_*", "*USB_Single_Serial*", "*"):
            matches = sorted(str(path) for path in by_id_dir.glob(pattern))
            if matches:
                return matches[0]

    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        matches = sorted(str(path) for path in Path("/dev").glob(pattern.removeprefix("/dev/")))
        if matches:
            return matches[0]

    return "/dev/ttyACM0"


@RobotConfig.register_subclass("lekiwi")
@dataclass
class LeKiwiConfig(RobotConfig):
    port: str = field(default_factory=default_lekiwi_port)  # port to connect to the bus

    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a dictionary that maps motor
    # names to the max_relative_target value for that motor.
    max_relative_target: float | dict[str, float] | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=lekiwi_cameras_config)

    # Runtime wheel tuning. `base_wheel_torque_limit` only touches the SRAM torque limit register, so
    # it is cleared on reboot and does not rewrite the persistent EEPROM safety limits.
    base_max_raw_velocity: int = 3000
    base_wheel_torque_limit: int | None = None

    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = False


@dataclass
class LeKiwiHostConfig:
    # Network Configuration
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    # Duration of the application
    connection_time_s: int = 30

    # Watchdog: stop the robot if no command is received for over 0.5 seconds.
    watchdog_timeout_ms: int = 500

    # If robot jitters decrease the frequency and monitor cpu load with `top` in cmd
    max_loop_freq_hz: int = 30


@RobotConfig.register_subclass("lekiwi_client")
@dataclass
class LeKiwiClientConfig(RobotConfig):
    # Network Configuration
    remote_ip: str
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # Movement
            "forward": "w",
            "backward": "s",
            "left": "a",
            "right": "d",
            "rotate_left": "z",
            "rotate_right": "x",
            # Speed control
            "speed_up": "r",
            "speed_down": "f",
            # quit teleop
            "quit": "q",
        }
    )

    cameras: dict[str, CameraConfig] = field(default_factory=lekiwi_cameras_config)

    slow_xy_speed: float = 0.1
    medium_xy_speed: float = 0.2
    fast_xy_speed: float = 0.3

    slow_theta_speed: float = 30
    medium_theta_speed: float = 60
    fast_theta_speed: float = 90
    default_speed_index: int = 0

    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5
