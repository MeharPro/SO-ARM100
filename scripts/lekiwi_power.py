from __future__ import annotations

import csv
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Any

DEFAULT_POWER_SAMPLE_HZ = 2.0
DEFAULT_POWER_PRINT_EVERY_S = 1.0
DEFAULT_POWER_LOG_DIR = "~/lekiwi-power-logs"
DEFAULT_POWER_VOLTAGE_V_PER_RAW = 0.1
# STS3215 memory-table references commonly document 6.5mA per current-count.
DEFAULT_POWER_CURRENT_MA_PER_RAW = 6.5


def add_power_monitor_args(parser: Any) -> Any:
    parser.add_argument("--disable-power-monitor", action="store_true")
    parser.add_argument("--power-sample-hz", type=float, default=DEFAULT_POWER_SAMPLE_HZ)
    parser.add_argument("--power-print-every-s", type=float, default=DEFAULT_POWER_PRINT_EVERY_S)
    parser.add_argument("--power-log-dir", default=DEFAULT_POWER_LOG_DIR)
    parser.add_argument("--power-num-retry", type=int, default=1)
    parser.add_argument("--power-voltage-v-per-raw", type=float, default=DEFAULT_POWER_VOLTAGE_V_PER_RAW)
    parser.add_argument("--power-current-ma-per-raw", type=float, default=DEFAULT_POWER_CURRENT_MA_PER_RAW)
    return parser


class PowerTelemetryLogger:
    def __init__(self, robot: Any, args: Any, logger: Any, source_name: str):
        self.robot = robot
        self.logger = logger
        self.source_name = source_name
        self.motors = list(robot.bus.motors.keys())
        self.enabled = (not getattr(args, "disable_power_monitor", False)) and getattr(
            args, "power_sample_hz", 0.0
        ) > 0.0
        self.sample_period_s = 1.0 / max(float(getattr(args, "power_sample_hz", DEFAULT_POWER_SAMPLE_HZ)), 1e-6)
        self.print_every_s = max(float(getattr(args, "power_print_every_s", DEFAULT_POWER_PRINT_EVERY_S)), 0.0)
        self.log_dir = Path(os.path.expanduser(str(getattr(args, "power_log_dir", DEFAULT_POWER_LOG_DIR))))
        self.num_retry = int(getattr(args, "power_num_retry", 1))
        self.voltage_v_per_raw = float(
            getattr(args, "power_voltage_v_per_raw", DEFAULT_POWER_VOLTAGE_V_PER_RAW)
        )
        self.current_ma_per_raw = float(
            getattr(args, "power_current_ma_per_raw", DEFAULT_POWER_CURRENT_MA_PER_RAW)
        )
        self.csv_file = None
        self.csv_writer = None
        self.csv_path: Path | None = None
        self.next_sample_s = 0.0
        self.next_print_s = 0.0
        self.last_sample: dict[str, Any] | None = None
        self._sync_read_warning_state: dict[str, bool] = {}
        self._read_warning_state: dict[tuple[str, str], bool] = {}

    def start(self) -> None:
        if not self.enabled:
            return

        self.log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
        self.csv_path = self.log_dir / f"{self.source_name}-power-{timestamp}.csv"
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self._fieldnames())
        self.csv_writer.writeheader()
        self.csv_file.flush()

        print(f"Power telemetry logging to {self.csv_path}", flush=True)
        print(
            "Power telemetry is an estimate of motor-side electrical draw only; Pi, cameras, and converters are not included.",
            flush=True,
        )

    def close(self) -> None:
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        if self.csv_path is not None:
            print(f"Power telemetry saved to {self.csv_path}", flush=True)

    def maybe_sample(self, *, force: bool = False) -> None:
        if not self.enabled or self.csv_writer is None:
            return

        now = time.monotonic()
        if not force and now < self.next_sample_s:
            return
        self.next_sample_s = now + self.sample_period_s

        sample = self._read_sample(now)
        if sample is None:
            return

        self.csv_writer.writerow(sample)
        if self.csv_file is not None:
            self.csv_file.flush()
        self.last_sample = sample

        if self.print_every_s > 0.0 and (force or now >= self.next_print_s):
            self.next_print_s = now + self.print_every_s
            print(self._format_summary(sample), flush=True)

    def _fieldnames(self) -> list[str]:
        fields = [
            "timestamp_iso",
            "monotonic_s",
            "motors_reporting",
            "avg_voltage_v",
            "min_voltage_v",
            "max_voltage_v",
            "total_current_ma_est",
            "total_power_w_est",
            "max_temp_c",
            "hottest_motor",
            "peak_current_motor",
            "peak_current_ma_est",
        ]
        for motor in self.motors:
            fields.extend(
                [
                    f"{motor}.voltage_raw",
                    f"{motor}.voltage_v",
                    f"{motor}.current_raw",
                    f"{motor}.current_ma_est",
                    f"{motor}.power_w_est",
                    f"{motor}.temperature_c",
                ]
            )
        return fields

    def _read_metric(self, data_name: str) -> dict[str, int | None]:
        try:
            values = self.robot.bus.sync_read(data_name, self.motors, normalize=False, num_retry=self.num_retry)
            self._sync_read_warning_state[data_name] = False
            return {motor: int(values[motor]) for motor in self.motors}
        except Exception as sync_exc:
            if not self._sync_read_warning_state.get(data_name):
                self.logger.warning("Power telemetry sync_read(%s) failed: %s", data_name, sync_exc)
                self._sync_read_warning_state[data_name] = True

        values: dict[str, int | None] = {}
        for motor in self.motors:
            try:
                values[motor] = int(self.robot.bus.read(data_name, motor, normalize=False, num_retry=self.num_retry))
                self._read_warning_state[(data_name, motor)] = False
            except Exception as read_exc:
                if not self._read_warning_state.get((data_name, motor)):
                    self.logger.warning("Power telemetry read(%s, %s) failed: %s", data_name, motor, read_exc)
                    self._read_warning_state[(data_name, motor)] = True
                values[motor] = None

        return values

    def _read_sample(self, monotonic_s: float) -> dict[str, Any] | None:
        voltage_raw = self._read_metric("Present_Voltage")
        current_raw = self._read_metric("Present_Current")
        temperature_raw = self._read_metric("Present_Temperature")

        row: dict[str, Any] = {
            "timestamp_iso": datetime.now().astimezone().isoformat(timespec="seconds"),
            "monotonic_s": round(monotonic_s, 3),
        }

        voltages_v: list[float] = []
        currents_ma: list[float] = []
        powers_w: list[float] = []
        peak_current_motor = ""
        peak_current_ma = -1.0
        hottest_motor = ""
        max_temp_c = -1.0
        motors_reporting = 0

        for motor in self.motors:
            v_raw = voltage_raw.get(motor)
            c_raw = current_raw.get(motor)
            t_raw = temperature_raw.get(motor)

            v_v = float(v_raw) * self.voltage_v_per_raw if v_raw is not None else None
            c_ma = abs(float(c_raw)) * self.current_ma_per_raw if c_raw is not None else None
            p_w = (v_v * c_ma / 1000.0) if v_v is not None and c_ma is not None else None

            row[f"{motor}.voltage_raw"] = v_raw if v_raw is not None else ""
            row[f"{motor}.voltage_v"] = round(v_v, 3) if v_v is not None else ""
            row[f"{motor}.current_raw"] = c_raw if c_raw is not None else ""
            row[f"{motor}.current_ma_est"] = round(c_ma, 3) if c_ma is not None else ""
            row[f"{motor}.power_w_est"] = round(p_w, 3) if p_w is not None else ""
            row[f"{motor}.temperature_c"] = t_raw if t_raw is not None else ""

            if v_v is not None:
                voltages_v.append(v_v)
            if c_ma is not None:
                currents_ma.append(c_ma)
                if c_ma > peak_current_ma:
                    peak_current_ma = c_ma
                    peak_current_motor = motor
            if p_w is not None:
                powers_w.append(p_w)
                motors_reporting += 1
            if t_raw is not None and float(t_raw) > max_temp_c:
                max_temp_c = float(t_raw)
                hottest_motor = motor

        row["motors_reporting"] = motors_reporting
        row["avg_voltage_v"] = round(sum(voltages_v) / len(voltages_v), 3) if voltages_v else ""
        row["min_voltage_v"] = round(min(voltages_v), 3) if voltages_v else ""
        row["max_voltage_v"] = round(max(voltages_v), 3) if voltages_v else ""
        row["total_current_ma_est"] = round(sum(currents_ma), 3) if currents_ma else ""
        row["total_power_w_est"] = round(sum(powers_w), 3) if powers_w else ""
        row["max_temp_c"] = round(max_temp_c, 1) if hottest_motor else ""
        row["hottest_motor"] = hottest_motor
        row["peak_current_motor"] = peak_current_motor
        row["peak_current_ma_est"] = round(peak_current_ma, 3) if peak_current_motor else ""
        return row

    def _format_summary(self, sample: dict[str, Any]) -> str:
        pieces = [f"[power] {sample['timestamp_iso']}"]
        pieces.append(f"motors={sample['motors_reporting']}/{len(self.motors)}")

        if sample["total_power_w_est"] != "":
            pieces.append(f"total≈{sample['total_power_w_est']:.1f}W")
        if sample["total_current_ma_est"] != "":
            pieces.append(f"current≈{sample['total_current_ma_est'] / 1000.0:.2f}A")
        if sample["avg_voltage_v"] != "":
            pieces.append(
                f"voltage≈{sample['avg_voltage_v']:.2f}V ({sample['min_voltage_v']:.2f}-{sample['max_voltage_v']:.2f})"
            )
        if sample["peak_current_motor"]:
            pieces.append(
                f"peak={sample['peak_current_motor']}@{sample['peak_current_ma_est'] / 1000.0:.2f}A"
            )
        if sample["hottest_motor"]:
            pieces.append(f"hottest={sample['hottest_motor']}@{sample['max_temp_c']:.0f}C")

        return " ".join(pieces)
