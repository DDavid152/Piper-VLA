#!/usr/bin/env python3

"""Capture read-only Piper master/follower mapping evidence for active calibration."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from lerobot_robot_piper import PiperRobot, PiperRobotConfig
from lerobot_robot_piper_active.calibration import (
    FEATURES,
    MIN_PASSIVE_SAMPLES,
    _load_jsonl,
    _validate_passive_mapping,
)
from lerobot_teleoperator_piper import (
    PiperMasterTeleoperator,
    PiperMasterTeleoperatorConfig,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ADAPTER_SERIAL = "002900225547571120343930"


class PassiveMappingCaptureError(RuntimeError):
    """The passive evidence could not be captured without violating a gate."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument("--expected-adapter-serial", default=DEFAULT_ADAPTER_SERIAL)
    parser.add_argument("--duration-s", type=float, default=90.0)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--master-target-timeout-s", type=float, default=30.0)
    parser.add_argument("--stable-samples", type=int, default=10)
    parser.add_argument("--stability-tolerance", type=float, default=0.05)
    parser.add_argument("--minimum-pose-change", type=float, default=0.1)
    parser.add_argument(
        "--publish-pending",
        type=Path,
        help=(
            "Revalidate and publish a .pending.jsonl retained by an earlier run. "
            "This recovery mode does not open CAN."
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "logs" / "piper_calibration" / "passive_mapping.jsonl",
    )
    return parser.parse_args()


def _validate_cli(args: argparse.Namespace) -> None:
    if not args.can_interface.strip():
        raise ValueError("--can-interface must not be empty")
    if not args.expected_adapter_serial.strip():
        raise ValueError("--expected-adapter-serial must not be empty")
    if not math.isfinite(args.duration_s) or args.duration_s <= 0:
        raise ValueError("--duration-s must be positive")
    if not math.isfinite(args.fps) or args.fps <= 0 or args.fps > 30:
        raise ValueError("--fps must be in (0, 30]")
    if (
        not math.isfinite(args.master_target_timeout_s)
        or not 1 <= args.master_target_timeout_s <= 120
    ):
        raise ValueError("--master-target-timeout-s must be in [1, 120]")
    if isinstance(args.stable_samples, bool) or not 1 <= args.stable_samples <= 20:
        raise ValueError("--stable-samples must be an integer in [1, 20]")
    if (
        not math.isfinite(args.stability_tolerance)
        or not 0 < args.stability_tolerance <= 0.5
    ):
        raise ValueError("--stability-tolerance must be in (0, 0.5]")
    if (
        not math.isfinite(args.minimum_pose_change)
        or not 0 < args.minimum_pose_change <= 5
    ):
        raise ValueError("--minimum-pose-change must be in (0, 5]")


def _counter_path(can_interface: str, name: str) -> Path:
    return Path("/sys/class/net") / can_interface / "statistics" / name


def read_can_counter(can_interface: str, name: str) -> int:
    path = _counter_path(can_interface, name)
    if not path.is_file():
        raise PassiveMappingCaptureError(f"CAN counter is unavailable: {path}")
    return int(path.read_text(encoding="utf-8").strip())


def _seven_values(values: dict[str, Any], *, label: str) -> list[float]:
    missing = set(FEATURES) - set(values)
    if missing:
        raise PassiveMappingCaptureError(f"{label} is missing features: {sorted(missing)}")
    result = [float(values[feature]) for feature in FEATURES]
    if not all(math.isfinite(value) for value in result):
        raise PassiveMappingCaptureError(f"{label} contains NaN or Inf")
    return result


def wait_for_master_targets(
    teleoperator: PiperMasterTeleoperator,
    *,
    timeout_s: float,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    """Wait for the dynamic target burst emitted when the physical master moves."""

    deadline = monotonic() + timeout_s
    while monotonic() < deadline:
        health = teleoperator.get_health_stats()
        if (
            health.get("action_source") == "master_target"
            and health.get("joint_target_received")
            and health.get("gripper_target_received")
        ):
            if int(health.get("error_frames", 0)) != 0:
                raise PassiveMappingCaptureError("An error CAN frame was observed.")
            print("First complete dynamic master target received; calibration capture is active.")
            return
        sleep(0.05)
    raise PassiveMappingCaptureError(
        "No complete dynamic master target arrived before timeout. Piper master target "
        "frames are emitted only while the physical master is moved; verify native "
        "master/follower operation and gently move the master once after capture starts."
    )


def capture_records(
    robot: PiperRobot,
    teleoperator: PiperMasterTeleoperator,
    *,
    adapter_serial: str,
    duration_s: float,
    fps: float,
    stable_samples: int = 10,
    stability_tolerance: float = 0.05,
    minimum_pose_change: float = 0.1,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> list[dict[str, Any]]:
    """Collect synchronized mappings; both supplied interfaces must remain receive-only."""

    period_s = 1.0 / fps
    started = monotonic()
    records: list[dict[str, Any]] = []
    last_recorded_master: list[float] | None = None
    stability_window: deque[list[float]] = deque(maxlen=stable_samples)
    interrupted = False
    try:
        while monotonic() - started < duration_s:
            iteration_started = monotonic()
            master_action = teleoperator.get_action()
            follower_observation = robot.get_observation()
            health = teleoperator.get_health_stats()
            if health.get("action_source") != "master_target":
                raise PassiveMappingCaptureError(
                    "Master target frames are unavailable; refusing follower-feedback fallback."
                )
            if not health.get("joint_target_received") or not health.get(
                "gripper_target_received"
            ):
                raise PassiveMappingCaptureError(
                    "Complete master joint and gripper targets are required."
                )
            if int(health.get("error_frames", 0)) != 0:
                raise PassiveMappingCaptureError("An error CAN frame was observed.")

            master = _seven_values(master_action, label="master action")
            follower = _seven_values(follower_observation, label="follower observation")
            stability_window.append([*master, *follower])
            stable = len(stability_window) == stable_samples and max(
                max(values) - min(values)
                for values in zip(*stability_window, strict=True)
            ) <= stability_tolerance

            distinct_pose = last_recorded_master is None or max(
                abs(value - previous)
                for value, previous in zip(master, last_recorded_master, strict=True)
            ) >= minimum_pose_change
            if stable and distinct_pose:
                captured_monotonic = monotonic()
                records.append(
                    {
                        "schema_version": 1,
                        "record_type": "piper_passive_mapping",
                        "capture_mode": "read_only",
                        "adapter_serial": adapter_serial,
                        "sequence": len(records),
                        "wall_time_utc": datetime.now(timezone.utc).isoformat(),
                        "monotonic_time_s": captured_monotonic,
                        "master": master,
                        "follower": follower,
                    }
                )
                last_recorded_master = master.copy()
                print(f"Accepted stable calibration pose {len(records)}.", flush=True)
            sleep(max(0.0, period_s - (monotonic() - iteration_started)))
    except KeyboardInterrupt:
        interrupted = True

    if interrupted:
        print("Capture stopped by operator; validating samples already collected.")
    if len(records) < MIN_PASSIVE_SAMPLES:
        raise PassiveMappingCaptureError(
            f"Only {len(records)} samples were captured; at least "
            f"{MIN_PASSIVE_SAMPLES} are required."
        )
    return records


def _write_jsonl(records: list[dict[str, Any]], path: Path) -> None:
    with path.open("w", encoding="utf-8") as stream:
        for record in records:
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        stream.flush()
        os.fsync(stream.fileno())


def validate_and_publish(
    records: list[dict[str, Any]],
    output: Path,
    *,
    expected_adapter_serial: str,
) -> dict[str, Any]:
    """Validate with the v2 generator's exact rules, then publish without overwriting."""

    output = output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite calibration evidence: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.stem}.", suffix=".pending.jsonl", dir=output.parent
    )
    os.close(descriptor)
    temporary_path = Path(temporary_name)
    _write_jsonl(records, temporary_path)
    try:
        _, joint_mappings, gripper_mapping = _validate_passive_mapping(
            temporary_path,
            expected_adapter_serial=expected_adapter_serial,
        )
        with output.open("x", encoding="utf-8") as stream:
            for record in records:
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        print(f"Unpublished evidence retained for diagnosis: {temporary_path}")
        raise
    else:
        temporary_path.unlink()

    return {
        "output": str(output),
        "samples": len(records),
        "joint_max_errors_degrees": [mapping["max_error"] for mapping in joint_mappings],
        "gripper_max_error_mm": gripper_mapping["max_error"],
        "master_spans": [
            max(record["master"][index] for record in records)
            - min(record["master"][index] for record in records)
            for index in range(7)
        ],
    }


def main() -> int:
    args = parse_args()
    _validate_cli(args)
    output = args.output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite calibration evidence: {output}")

    if args.publish_pending is not None:
        pending = args.publish_pending.expanduser().resolve()
        expected_prefix = f".{output.stem}."
        if (
            pending.parent != output.parent
            or not pending.name.startswith(expected_prefix)
            or not pending.name.endswith(".pending.jsonl")
            or not pending.is_file()
        ):
            raise PassiveMappingCaptureError(
                "--publish-pending must name this output's retained "
                f"{expected_prefix}*.pending.jsonl file in {output.parent}."
            )
        records = _load_jsonl(pending)
        report = validate_and_publish(
            records,
            output,
            expected_adapter_serial=args.expected_adapter_serial,
        )
        report["recovered_from"] = str(pending)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print(
            "PASS: retained passive mapping evidence revalidated and published; "
            "CAN was not opened in recovery mode."
        )
        return 0

    tx_before = read_can_counter(args.can_interface, "tx_packets")
    rx_before = read_can_counter(args.can_interface, "rx_packets")
    robot = PiperRobot(
        PiperRobotConfig(
            can_interface=args.can_interface,
            expected_adapter_serial=args.expected_adapter_serial,
            cameras={},
        )
    )
    teleoperator = PiperMasterTeleoperator(
        PiperMasterTeleoperatorConfig(
            can_interface=args.can_interface,
            expected_adapter_serial=args.expected_adapter_serial,
        )
    )
    records: list[dict[str, Any]] | None = None
    try:
        robot.connect(calibrate=False)
        teleoperator.connect(calibrate=False)
        print(
            "Read-only capture connected. Gently move the physical master once so its "
            "dynamic target frames become visible; no sample is recorded before that.",
            flush=True,
        )
        wait_for_master_targets(
            teleoperator,
            timeout_s=args.master_target_timeout_s,
        )
        print(
            "Move through official native master/follower teleoperation; pause at each "
            "distinct pose, cover all joints and the full gripper range. Ctrl+C may "
            "finish early."
        )
        records = capture_records(
            robot,
            teleoperator,
            adapter_serial=args.expected_adapter_serial,
            duration_s=args.duration_s,
            fps=args.fps,
            stable_samples=args.stable_samples,
            stability_tolerance=args.stability_tolerance,
            minimum_pose_change=args.minimum_pose_change,
        )
    finally:
        teleoperator.disconnect()
        if robot.is_connected:
            robot.disconnect()

    tx_after = read_can_counter(args.can_interface, "tx_packets")
    rx_after = read_can_counter(args.can_interface, "rx_packets")
    if tx_after != tx_before:
        raise PassiveMappingCaptureError(
            f"Host CAN TX changed from {tx_before} to {tx_after}; evidence was not published."
        )
    if rx_after <= rx_before:
        raise PassiveMappingCaptureError("CAN RX did not increase; evidence was not published.")
    if records is None:
        raise PassiveMappingCaptureError("No records were captured.")

    report = validate_and_publish(
        records,
        output,
        expected_adapter_serial=args.expected_adapter_serial,
    )
    report["can"] = {
        "tx_before": tx_before,
        "tx_after": tx_after,
        "rx_before": rx_before,
        "rx_after": rx_after,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print("PASS: passive mapping evidence published with zero host CAN TX.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (PassiveMappingCaptureError, FileExistsError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from None
