#!/usr/bin/env python3

from __future__ import annotations

import glob
import os
import shutil
from pathlib import Path
from typing import Any


def detect_leader_port(requested_port: str | None) -> str:
    if requested_port and requested_port.lower() != "auto":
        return requested_port

    candidates = sorted(
        set(glob.glob("/dev/tty.usbmodem*"))
        | set(glob.glob("/dev/tty.usbserial*"))
        | set(glob.glob("/dev/cu.usbmodem*"))
        | set(glob.glob("/dev/cu.usbserial*"))
    )
    if not candidates:
        raise SystemExit(
            "No leader-arm serial port found. Plug in the leader arm and rerun start control."
        )

    grouped: dict[str, list[str]] = {}
    for port in candidates:
        name = os.path.basename(port)
        if name.startswith("tty."):
            key = name.removeprefix("tty.")
        elif name.startswith("cu."):
            key = name.removeprefix("cu.")
        else:
            key = name
        grouped.setdefault(key, []).append(port)

    deduped = []
    for key in sorted(grouped):
        options = sorted(grouped[key])
        tty_port = next((item for item in options if os.path.basename(item).startswith("tty.")), None)
        deduped.append(tty_port or options[0])

    if len(deduped) == 1:
        return deduped[0]

    raise SystemExit(
        "More than one leader-arm serial port was found. Pass --leader-port explicitly.\n"
        + "\n".join(deduped)
    )


def _leader_calibration_aliases(leader_id: str | None) -> tuple[str, ...]:
    normalized = (leader_id or "leader").strip() or "leader"
    aliases = [normalized]
    if normalized == "leader":
        aliases.append("lead")
    elif normalized == "lead":
        aliases.append("leader")
    return tuple(dict.fromkeys(aliases))


def _bundled_calibration_candidates(leader_id: str | None) -> list[Path]:
    assets_dir = (
        Path(__file__).resolve().parents[1]
        / "preserved"
        / "lekiwi-home-kit"
        / "assets"
        / "calibration"
        / "teleoperators"
        / "so_leader"
    )
    return [assets_dir / f"{alias}.json" for alias in _leader_calibration_aliases(leader_id)]


def ensure_leader_calibration_file(leader) -> Path | None:
    if leader.calibration_fpath.is_file():
        return leader.calibration_fpath

    for candidate in _bundled_calibration_candidates(getattr(leader, "id", None)):
        if not candidate.is_file():
            continue

        leader.calibration_fpath.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(candidate, leader.calibration_fpath)
        leader._load_calibration()
        print(
            f"Installed bundled leader calibration from {candidate} to {leader.calibration_fpath}.",
            flush=True,
        )
        return candidate

    return None


def connect_leader_noninteractive(leader, calibrate_hint: str) -> None:
    ensure_leader_calibration_file(leader)

    if not leader.calibration:
        raise RuntimeError(
            "Leader arm calibration is required before teleop can start. "
            f"No calibration file was found for '{leader.id}' at {leader.calibration_fpath}. "
            f"{calibrate_hint}"
        )

    leader.connect(calibrate=False)
    if leader.is_calibrated:
        return

    print(f"Applying cached leader calibration from {leader.calibration_fpath}.", flush=True)
    leader.bus.write_calibration(leader.calibration)
    leader.configure()

    if leader.is_calibrated:
        return

    raise RuntimeError(
        "Leader arm calibration does not match the connected hardware even after applying the cached values. "
        f"Re-run calibration. {calibrate_hint}"
    )


def disconnect_device_safely(device: Any, label: str) -> None:
    if device is None:
        return

    try:
        is_connected = bool(getattr(device, "is_connected", False))
    except Exception as exc:
        print(f"Warning: could not inspect {label} connection state during shutdown: {exc}", flush=True)
        is_connected = True

    if not is_connected:
        return

    try:
        device.disconnect()
    except Exception as exc:
        print(f"Warning: {label} disconnect failed during shutdown: {exc}", flush=True)
