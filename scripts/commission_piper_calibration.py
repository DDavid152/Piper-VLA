#!/usr/bin/env python3

"""Run the fixed Piper +/-0.5 degree commissioning used to generate calibration v2."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import select
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import can
from piper_sdk import C_PiperInterface_V2

from lerobot_robot_piper.robot_piper import (
    GRIPPER_ERROR_FIELDS,
    read_usb_serial_for_network_interface,
)
from lerobot_robot_piper_active.calibration import (
    COMMISSIONING_DELTA_DEG,
    COMMISSIONING_MAX_ERROR_DEG,
    PIPER_JOINT_MAX_DEG,
    PIPER_JOINT_MIN_DEG,
    _validate_commissioning,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ADAPTER_SERIAL = "002900225547571120343930"
DEFAULT_BASELINE = PROJECT_ROOT / "config" / "piper_safety_baseline_v1.json"
MOTION_CONFIRMATION = "I_UNDERSTAND_PIPER_COMMISSIONING_WILL_MOVE"
START_CONFIRMATION = "START_12_FIXED_COMMISSIONING_TESTS"
ARM_ACQUISITION_MAX_DRIFT_DEG = 1.0
ARM_ACQUISITION_STABILITY_DEG = 0.05
MASTER_COMMAND_IDS = frozenset((0x155, 0x156, 0x157, 0x159))
CAN_DETAIL_COUNTERS = (
    "re-started",
    "bus-errors",
    "arbit-lost",
    "error-warn",
    "error-pass",
    "bus-off",
)


class CommissioningError(RuntimeError):
    """A commissioning safety or evidence gate failed."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument("--expected-adapter-serial", default=DEFAULT_ADAPTER_SERIAL)
    parser.add_argument("--safety-baseline", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "logs" / "piper_calibration" / "commissioning.jsonl",
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help=(
            "Run receive-only SDK/CAN/feedback/pose checks. This mode never enables "
            "the arm and never publishes commissioning evidence."
        ),
    )
    parser.add_argument(
        "--deployment-preflight",
        action="store_true",
        help=(
            "With --preflight-only, retain the training-start-envelope and all "
            "receive-only transport/feedback gates but do not require +/-0.5 "
            "degree commissioning margin. Use this only after commissioning is complete."
        ),
    )
    parser.add_argument(
        "--current-pose-deployment-preflight",
        action="store_true",
        help=(
            "With --preflight-only, require fresh feedback, physical joint limits, "
            "adapter identity, quiet external control, and zero host CAN TX without "
            "requiring the training start envelope or commissioning jog margin."
        ),
    )
    parser.add_argument("--operator", default="")
    parser.add_argument("--operator-confirmation", default="")
    parser.add_argument(
        "--manual-step",
        action="store_true",
        help=(
            "Use terminal commands 1+/1- through 6+/6- for bounded 0.5 degree "
            "setup jogs, then `c` locks an arbitrary physical commissioning "
            "baseline before the 12 fixed evidence tests."
        ),
    )
    parser.add_argument("--motion-speed-percent", type=int, default=5)
    parser.add_argument("--command-hz", type=float, default=20.0)
    parser.add_argument("--enable-timeout-s", type=float, default=5.0)
    parser.add_argument("--baseline-hold-s", type=float, default=0.5)
    parser.add_argument(
        "--max-jog-displacement-deg",
        type=float,
        default=5.0,
        help=(
            "Maximum absolute displacement of each joint from the pose measured "
            "when manual commissioning starts. Applies only before `c` locks the "
            "commissioning baseline."
        ),
    )
    parser.add_argument("--motion-timeout-s", type=float, default=3.0)
    parser.add_argument("--return-timeout-s", type=float, default=3.0)
    parser.add_argument("--inter-test-pause-s", type=float, default=1.0)
    parser.add_argument("--external-control-quiet-s", type=float, default=2.0)
    return parser.parse_args()


def _validate_cli(args: argparse.Namespace) -> None:
    if not args.can_interface.strip() or not args.expected_adapter_serial.strip():
        raise ValueError("CAN interface and adapter serial are required")
    if args.deployment_preflight and not args.preflight_only:
        raise ValueError("--deployment-preflight requires --preflight-only")
    if args.deployment_preflight and args.manual_step:
        raise ValueError("--deployment-preflight cannot be combined with --manual-step")
    if args.current_pose_deployment_preflight and not args.preflight_only:
        raise ValueError("--current-pose-deployment-preflight requires --preflight-only")
    if args.current_pose_deployment_preflight and (
        args.deployment_preflight or args.manual_step
    ):
        raise ValueError(
            "--current-pose-deployment-preflight cannot be combined with "
            "--deployment-preflight or --manual-step"
        )
    if not args.preflight_only:
        if not args.operator.strip():
            raise ValueError("--operator must not be empty")
        if args.operator_confirmation != MOTION_CONFIRMATION:
            raise ValueError(
                f"--operator-confirmation must be exactly {MOTION_CONFIRMATION!r}"
            )
    if isinstance(args.motion_speed_percent, bool) or not 1 <= args.motion_speed_percent <= 10:
        raise ValueError("--motion-speed-percent must be an integer in [1, 10]")
    if not math.isfinite(args.command_hz) or not 5 <= args.command_hz <= 50:
        raise ValueError("--command-hz must be in [5, 50]")
    if (
        not math.isfinite(args.max_jog_displacement_deg)
        or not 0.5 <= args.max_jog_displacement_deg <= 20
    ):
        raise ValueError("--max-jog-displacement-deg must be in [0.5, 20]")
    for name in (
        "enable_timeout_s",
        "baseline_hold_s",
        "motion_timeout_s",
        "return_timeout_s",
    ):
        value = float(getattr(args, name))
        if not math.isfinite(value) or not 0.5 <= value <= 10:
            raise ValueError(f"--{name.replace('_', '-')} must be in [0.5, 10]")
    if not math.isfinite(args.inter_test_pause_s) or not 0.5 <= args.inter_test_pause_s <= 10:
        raise ValueError("--inter-test-pause-s must be in [0.5, 10]")
    if (
        not math.isfinite(args.external_control_quiet_s)
        or not 2 <= args.external_control_quiet_s <= 10
    ):
        raise ValueError("--external-control-quiet-s must be in [2, 10]")
    if not args.safety_baseline.expanduser().is_file():
        raise ValueError(f"Safety baseline is missing: {args.safety_baseline}")


def parse_can_details(details: str) -> dict[str, int | str]:
    if "can state ERROR-ACTIVE" not in details:
        raise CommissioningError("CAN interface is not ERROR-ACTIVE.")
    if re.search(r"\bbitrate\s+1000000\b", details) is None:
        raise CommissioningError("CAN interface is not configured at 1 Mbps.")
    lines = details.splitlines()
    counters: dict[str, int | str] = {"state": "ERROR-ACTIVE", "bitrate": 1000000}
    for index, line in enumerate(lines[:-1]):
        if all(label in line for label in CAN_DETAIL_COUNTERS):
            values = lines[index + 1].split()
            if len(values) < len(CAN_DETAIL_COUNTERS):
                break
            for label, value in zip(
                CAN_DETAIL_COUNTERS, values[: len(CAN_DETAIL_COUNTERS)], strict=True
            ):
                counters[label] = int(value)
            return counters
    raise CommissioningError("Could not parse SocketCAN protocol error counters.")


def read_can_health(can_interface: str) -> dict[str, int | str]:
    completed = subprocess.run(
        ["ip", "-details", "-statistics", "link", "show", can_interface],
        check=True,
        capture_output=True,
        text=True,
    )
    health = parse_can_details(completed.stdout)
    for name in ("tx_packets", "rx_packets", "tx_errors", "rx_errors"):
        path = Path("/sys/class/net") / can_interface / "statistics" / name
        if not path.is_file():
            raise CommissioningError(f"CAN counter is unavailable: {path}")
        health[name] = int(path.read_text(encoding="utf-8").strip())
    return health


def can_error_delta(before: dict[str, int | str], after: dict[str, int | str]) -> int:
    names = (*CAN_DETAIL_COUNTERS, "tx_errors", "rx_errors")
    total = 0
    for name in names:
        earlier = int(before[name])
        later = int(after[name])
        if later < earlier:
            raise CommissioningError(f"CAN counter {name} regressed from {earlier} to {later}.")
        total += later - earlier
    return total


def ensure_external_control_quiet(
    can_interface: str,
    duration_s: float,
    *,
    bus_factory: Callable[..., Any] = can.Bus,
    monotonic: Callable[[], float] = time.monotonic,
) -> None:
    bus = bus_factory(
        interface="socketcan",
        channel=can_interface,
        receive_own_messages=False,
        can_filters=[
            {"can_id": identifier, "can_mask": 0x7FF, "extended": False}
            for identifier in MASTER_COMMAND_IDS
        ],
    )
    deadline = monotonic() + duration_s
    try:
        while monotonic() < deadline:
            message = bus.recv(timeout=min(0.05, max(0.0, deadline - monotonic())))
            if message is not None and int(message.arbitration_id) in MASTER_COMMAND_IDS:
                raise CommissioningError(
                    f"External master command frame 0x{message.arbitration_id:03X} detected."
                )
    finally:
        bus.shutdown()


def _validate_wrapper(name: str, wrapper: Any, *, wall_time: float) -> None:
    timestamp = float(wrapper.time_stamp)
    if timestamp <= 0 or wall_time - timestamp < -0.1 or wall_time - timestamp > 0.25:
        raise CommissioningError(f"{name} feedback is stale or invalid.")
    if float(wrapper.Hz) < 20:
        raise CommissioningError(f"{name} feedback is below 20 Hz.")


def read_joint_feedback(
    interface: Any,
    *,
    require_normal_status: bool = True,
    wall_clock: Callable[[], float] = time.time,
) -> list[float]:
    if not interface.get_connect_status() or not interface.isOk():
        raise CommissioningError("Piper SDK connection or CAN receive thread is unhealthy.")
    status = interface.GetArmStatus()
    joints = interface.GetArmJointMsgs()
    gripper = interface.GetArmGripperMsgs()
    now = wall_clock()
    _validate_wrapper("arm status", status, wall_time=now)
    _validate_wrapper("joint", joints, wall_time=now)
    _validate_wrapper("gripper", gripper, wall_time=now)
    arm_status = status.arm_status
    if require_normal_status and (
        int(arm_status.arm_status) != 0 or int(arm_status.err_code) != 0
    ):
        raise CommissioningError(
            f"Piper reports arm_status={arm_status.arm_status}, err_code={arm_status.err_code}."
        )
    foc_status = gripper.gripper_state.foc_status
    gripper_errors = [
        field for field in GRIPPER_ERROR_FIELDS if bool(getattr(foc_status, field))
    ]
    if gripper_errors:
        raise CommissioningError(f"Piper gripper reports errors: {gripper_errors}.")
    joint_state = joints.joint_state
    values = [
        float(getattr(joint_state, f"joint_{index}")) * 0.001
        for index in range(1, 7)
    ]
    if not all(math.isfinite(value) for value in values):
        raise CommissioningError("Joint feedback contains NaN or Inf.")
    for index, value in enumerate(values):
        if not PIPER_JOINT_MIN_DEG[index] <= value <= PIPER_JOINT_MAX_DEG[index]:
            raise CommissioningError(f"joint_{index + 1} feedback is outside a physical limit.")
    return values


def wait_for_ready_feedback(
    interface: Any,
    *,
    timeout_s: float = 3.0,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> list[float]:
    deadline = monotonic() + timeout_s
    last_error: Exception | None = None
    while monotonic() < deadline:
        try:
            return read_joint_feedback(interface)
        except CommissioningError as exc:
            last_error = exc
            sleep(0.05)
    detail = f" Last error: {last_error}" if last_error is not None else ""
    raise CommissioningError(
        f"Fresh, healthy Piper feedback did not arrive.{detail}"
    ) from last_error


def validate_commissioning_pose(
    joints: list[float],
    baseline: dict[str, Any],
    *,
    require_training_envelope: bool = True,
    require_bidirectional_margin: bool = True,
) -> None:
    if require_training_envelope:
        initial = baseline["initial_state"]
        lower = [float(value) for value in initial["min"][:6]]
        upper = [float(value) for value in initial["max"][:6]]
        tolerance = [float(value) for value in initial["tolerance"][:6]]
        for index, value in enumerate(joints):
            if (
                value < lower[index] - tolerance[index]
                or value > upper[index] + tolerance[index]
            ):
                raise CommissioningError(
                    f"joint_{index + 1} is outside the training start envelope."
                )
    if require_bidirectional_margin:
        for index, value in enumerate(joints):
            if value - COMMISSIONING_DELTA_DEG < PIPER_JOINT_MIN_DEG[index] or (
                value + COMMISSIONING_DELTA_DEG > PIPER_JOINT_MAX_DEG[index]
            ):
                raise CommissioningError(
                    f"joint_{index + 1} lacks physical margin for both +/-0.5 degree tests."
                )


def receive_only_preflight(
    interface: Any,
    baseline: dict[str, Any],
    *,
    require_training_envelope: bool = True,
    require_bidirectional_margin: bool = True,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> list[float]:
    """Validate fresh feedback and the commissioning pose without control calls."""

    joints = wait_for_ready_feedback(
        interface,
        monotonic=monotonic,
        sleep=sleep,
    )
    validate_commissioning_pose(
        joints,
        baseline,
        require_training_envelope=require_training_envelope,
        require_bidirectional_margin=require_bidirectional_margin,
    )
    return joints


def _joint_units(joints: list[float]) -> list[int]:
    return [int(round(value * 1000.0)) for value in joints]


def _stream_joint_target(
    interface: Any,
    target: list[float],
    *,
    motion_speed_percent: int,
) -> None:
    """Send one official MOVE J mode/target refresh."""

    interface.ModeCtrl(1, 1, motion_speed_percent, 0)
    interface.JointCtrl(*_joint_units(target))


def wait_until_enabled(
    interface: Any,
    *,
    timeout_s: float,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    """Use the SDK wrapper until all six motor-enable feedback bits are true."""

    deadline = monotonic() + timeout_s
    while monotonic() < deadline:
        if bool(interface.EnablePiper()):
            return
        sleep(0.01)
    raise CommissioningError(
        f"All six Piper motors were not enabled within {timeout_s:.1f}s."
    )


def validate_baseline_hold(
    interface: Any,
    baseline: list[float],
    *,
    hold_s: float,
    command_hz: float,
    motion_speed_percent: int,
    max_departure_deg: float = COMMISSIONING_MAX_ERROR_DEG,
    stability_tolerance_deg: float = ARM_ACQUISITION_STABILITY_DEG,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> list[float]:
    """Acquire and continuously hold the measured start pose before any test."""

    period_s = 1.0 / command_hz
    deadline = monotonic() + hold_s
    stable_samples = 0
    last_state = baseline
    while monotonic() < deadline or stable_samples < 3:
        iteration_started = monotonic()
        _stream_joint_target(
            interface,
            baseline,
            motion_speed_percent=motion_speed_percent,
        )
        sleep(max(0.0, period_s - (monotonic() - iteration_started)))
        state = read_joint_feedback(interface)
        deltas = [
            actual - origin
            for actual, origin in zip(state, baseline, strict=True)
        ]
        max_departure = max(abs(value) for value in deltas)
        if max_departure > max_departure_deg:
            raise CommissioningError(
                "Robot departed from the commissioning baseline while acquiring "
                f"the hold; baseline={baseline}, feedback={state}, deltas={deltas}, "
                f"allowed_departure={max_departure_deg:.3f}."
            )
        sample_change = max(
            abs(actual - previous)
            for actual, previous in zip(state, last_state, strict=True)
        )
        if sample_change <= stability_tolerance_deg:
            stable_samples += 1
        else:
            stable_samples = 0
        last_state = state
    return last_state


def drive_to_target(
    interface: Any,
    target: list[float],
    *,
    timeout_s: float,
    command_hz: float,
    motion_speed_percent: int,
    reference: list[float] | None = None,
    moving_joint_index: int | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> list[float]:
    for index, value in enumerate(target):
        if not PIPER_JOINT_MIN_DEG[index] <= value <= PIPER_JOINT_MAX_DEG[index]:
            raise CommissioningError(f"joint_{index + 1} target is outside a physical limit.")
    period_s = 1.0 / command_hz
    deadline = monotonic() + timeout_s
    stable_samples = 0
    last_state: list[float] | None = None
    while monotonic() < deadline:
        iteration_started = monotonic()
        _stream_joint_target(
            interface,
            target,
            motion_speed_percent=motion_speed_percent,
        )
        sleep(max(0.0, period_s - (monotonic() - iteration_started)))
        state = read_joint_feedback(interface)
        if reference is not None and moving_joint_index is not None:
            other_drift = max(
                abs(state[index] - reference[index])
                for index in range(6)
                if index != moving_joint_index
            )
            if other_drift > COMMISSIONING_MAX_ERROR_DEG:
                raise CommissioningError(
                    f"Another joint drifted {other_drift:.3f} degrees during commissioning."
                )
        error = max(
            abs(actual - requested)
            for actual, requested in zip(state, target, strict=True)
        )
        if error <= COMMISSIONING_MAX_ERROR_DEG:
            stable_samples += 1
            if stable_samples >= 3:
                return state
        else:
            stable_samples = 0
        last_state = state
    raise CommissioningError(
        f"Target was not reached within {timeout_s:.1f}s; last feedback={last_state}."
    )


def wait_for_manual_command(
    interface: Any,
    hold_target: list[float],
    *,
    prompt: str,
    allowed_commands: set[str],
    motion_speed_percent: int,
    command_hz: float,
    can_error_reader: Callable[[], int],
    error_count_before: int,
    command_reader: Callable[[], str] | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> str:
    """Hold a target while waiting for a bounded terminal command."""

    print(prompt, flush=True)
    period_s = 1.0 / command_hz
    while True:
        iteration_started = monotonic()
        _stream_joint_target(
            interface,
            hold_target,
            motion_speed_percent=motion_speed_percent,
        )
        if command_reader is None:
            ready, _, _ = select.select(
                [sys.stdin],
                [],
                [],
                max(0.0, period_s - (monotonic() - iteration_started)),
            )
            raw_command = sys.stdin.readline() if ready else ""
            if ready and raw_command == "":
                raise CommissioningError("Commissioning terminal input closed unexpectedly.")
        else:
            sleep(max(0.0, period_s - (monotonic() - iteration_started)))
            raw_command = command_reader()

        state = read_joint_feedback(interface)
        hold_error = max(
            abs(actual - requested)
            for actual, requested in zip(state, hold_target, strict=True)
        )
        if hold_error > COMMISSIONING_MAX_ERROR_DEG:
            raise CommissioningError(
                "Piper did not hold the requested interactive pose; "
                f"target={hold_target}, feedback={state}, max_error={hold_error:.3f}."
            )
        current_error_count = can_error_reader() - error_count_before
        if current_error_count != 0:
            raise CommissioningError(
                f"CAN error count changed by {current_error_count} while waiting for input."
            )
        if not raw_command:
            continue
        command = raw_command.strip().lower()
        if command == "q":
            raise CommissioningError("Operator aborted interactive commissioning.")
        if command == "s":
            formatted = ", ".join(
                f"J{index}={value:.3f}°" for index, value in enumerate(state, 1)
            )
            print(f"Feedback: {formatted}", flush=True)
            continue
        if command in allowed_commands:
            return command
        print(
            f"Ignored command {command!r}; allowed: "
            f"{', '.join(sorted(allowed_commands))}, s, q",
            flush=True,
        )


def _send_emergency_stop_once(interface: Any) -> None:
    interface.EmergencyStop(1)


def verify_emergency_stop_feedback(
    interface: Any,
    *,
    timeout_s: float = 1.0,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    deadline = monotonic() + timeout_s
    while monotonic() < deadline:
        status = interface.GetArmStatus()
        _validate_wrapper("arm status", status, wall_time=time.time())
        if int(status.arm_status.arm_status) == 1:
            return
        sleep(0.05)
    raise CommissioningError(
        "EmergencyStop(1) was sent but emergency-stop feedback was not observed."
    )


def execute_commissioning(
    interface: Any,
    *,
    baseline: dict[str, Any],
    adapter_serial: str,
    operator: str,
    motion_speed_percent: int,
    command_hz: float,
    enable_timeout_s: float = 5.0,
    baseline_hold_s: float = 0.5,
    max_jog_displacement_deg: float = 5.0,
    motion_timeout_s: float,
    return_timeout_s: float,
    inter_test_pause_s: float,
    can_error_reader: Callable[[], int],
    manual_step: bool = False,
    command_reader: Callable[[], str] | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> list[dict[str, Any]]:
    """Execute all 12 fixed tests and always latch EmergencyStop after an arm attempt."""

    session_start = wait_for_ready_feedback(
        interface, monotonic=monotonic, sleep=sleep
    )
    validate_commissioning_pose(
        session_start,
        baseline,
        require_training_envelope=not manual_step,
        require_bidirectional_margin=not manual_step,
    )
    error_count_before = can_error_reader()
    records: list[dict[str, Any]] = []
    arm_attempted = False
    stop_sent = False
    failure: BaseException | None = None
    stop_failure: BaseException | None = None

    try:
        arm_attempted = True
        wait_until_enabled(
            interface,
            timeout_s=enable_timeout_s,
            monotonic=monotonic,
            sleep=sleep,
        )
        measured_before_arm = session_start.copy()
        settled_state = validate_baseline_hold(
            interface,
            session_start,
            hold_s=baseline_hold_s,
            command_hz=command_hz,
            motion_speed_percent=motion_speed_percent,
            max_departure_deg=ARM_ACQUISITION_MAX_DRIFT_DEG,
            monotonic=monotonic,
            sleep=sleep,
        )
        settling_deltas = [
            settled - measured
            for settled, measured in zip(
                settled_state,
                measured_before_arm,
                strict=True,
            )
        ]
        session_start = settled_state
        validate_commissioning_pose(
            session_start,
            baseline,
            require_training_envelope=not manual_step,
            require_bidirectional_margin=not manual_step,
        )
        validate_baseline_hold(
            interface,
            session_start,
            hold_s=baseline_hold_s,
            command_hz=command_hz,
            motion_speed_percent=motion_speed_percent,
            max_departure_deg=COMMISSIONING_MAX_ERROR_DEG,
            monotonic=monotonic,
            sleep=sleep,
        )
        if max(abs(value) for value in settling_deltas) > 1e-9:
            print(
                "Arm-enable settling accepted and the hold target was rebased: "
                f"deltas={settling_deltas}.",
                flush=True,
            )

        if manual_step:
            jog_origin = session_start.copy()
            jog_target = session_start.copy()
            jog_commands = {
                f"{joint}{sign}"
                for joint in range(1, 7)
                for sign in ("-", "+")
            }
            while True:
                command = wait_for_manual_command(
                    interface,
                    jog_target,
                    prompt=(
                        "JOG SETUP: enter 1+/1- through 6+/6- for one 0.5° step; "
                        "c=lock the current pose as the commissioning baseline, "
                        "s=status, q=abort:"
                    ),
                    allowed_commands=jog_commands | {"c"},
                    motion_speed_percent=motion_speed_percent,
                    command_hz=command_hz,
                    can_error_reader=can_error_reader,
                    error_count_before=error_count_before,
                    command_reader=command_reader,
                    monotonic=monotonic,
                    sleep=sleep,
                )
                if command == "c":
                    candidate = read_joint_feedback(interface)
                    try:
                        validate_commissioning_pose(
                            candidate,
                            baseline,
                            require_training_envelope=False,
                            require_bidirectional_margin=True,
                        )
                    except CommissioningError as exc:
                        print(
                            f"Cannot lock this baseline yet: {exc}",
                            flush=True,
                        )
                        continue
                    session_start = candidate
                    validate_baseline_hold(
                        interface,
                        session_start,
                        hold_s=baseline_hold_s,
                        command_hz=command_hz,
                        motion_speed_percent=motion_speed_percent,
                        monotonic=monotonic,
                        sleep=sleep,
                    )
                    print(
                        "Commissioning baseline locked at: "
                        + ", ".join(
                            f"J{index}={value:.3f}°"
                            for index, value in enumerate(session_start, 1)
                        ),
                        flush=True,
                    )
                    break

                joint_index = int(command[0]) - 1
                requested_delta = (
                    COMMISSIONING_DELTA_DEG
                    if command[1] == "+"
                    else -COMMISSIONING_DELTA_DEG
                )
                target = jog_target.copy()
                target[joint_index] += requested_delta
                if not (
                    PIPER_JOINT_MIN_DEG[joint_index]
                    <= target[joint_index]
                    <= PIPER_JOINT_MAX_DEG[joint_index]
                ):
                    print(
                        f"Rejected: J{joint_index + 1} target "
                        f"{target[joint_index]:.3f}° exceeds its physical limit.",
                        flush=True,
                    )
                    continue
                session_displacement = abs(
                    target[joint_index] - jog_origin[joint_index]
                )
                if session_displacement > max_jog_displacement_deg + 1e-9:
                    print(
                        f"Rejected: J{joint_index + 1} would exceed the "
                        f"{max_jog_displacement_deg:.1f}° session jog window.",
                        flush=True,
                    )
                    continue
                reached = drive_to_target(
                    interface,
                    target,
                    timeout_s=motion_timeout_s,
                    command_hz=command_hz,
                    motion_speed_percent=motion_speed_percent,
                    reference=jog_target,
                    moving_joint_index=joint_index,
                    monotonic=monotonic,
                    sleep=sleep,
                )
                current_error_count = can_error_reader() - error_count_before
                if current_error_count != 0:
                    raise CommissioningError(
                        f"CAN error count changed by {current_error_count} during manual jog."
                    )
                jog_target = target
                print(
                    f"JOG J{joint_index + 1} {requested_delta:+.1f}° reached; "
                    f"feedback={reached[joint_index]:.3f}°.",
                    flush=True,
                )

        def run_one_test(joint_index: int, requested_delta: float) -> list[float]:
            current = read_joint_feedback(interface)
            deltas_from_start = [
                value - origin
                for value, origin in zip(current, session_start, strict=True)
            ]
            if max(abs(value) for value in deltas_from_start) > COMMISSIONING_MAX_ERROR_DEG:
                raise CommissioningError(
                    "Robot is not at the commissioning baseline before the next test; "
                    f"baseline={session_start}, feedback={current}, "
                    f"deltas={deltas_from_start}."
                )
            target = session_start.copy()
            target[joint_index] += requested_delta
            print(
                f"TEST {len(records) + 1}/12: joint_{joint_index + 1} "
                f"{requested_delta:+.1f} degree",
                flush=True,
            )
            reached = drive_to_target(
                interface,
                target,
                timeout_s=motion_timeout_s,
                command_hz=command_hz,
                motion_speed_percent=motion_speed_percent,
                reference=session_start,
                moving_joint_index=joint_index,
                monotonic=monotonic,
                sleep=sleep,
            )
            measured_delta = reached[joint_index] - session_start[joint_index]
            other_drift = max(
                abs(reached[index] - session_start[index])
                for index in range(6)
                if index != joint_index
            )
            direction = 1 if requested_delta > 0 else -1
            if measured_delta * direction <= 0 or abs(
                measured_delta - requested_delta
            ) > COMMISSIONING_MAX_ERROR_DEG:
                raise CommissioningError(
                    f"joint_{joint_index + 1} measured {measured_delta:+.3f} degrees "
                    f"for request {requested_delta:+.3f}."
                )
            if other_drift > COMMISSIONING_MAX_ERROR_DEG:
                raise CommissioningError(
                    f"Other-joint drift {other_drift:.3f} degrees exceeds tolerance."
                )
            current_error_count = can_error_reader() - error_count_before
            if current_error_count != 0:
                raise CommissioningError(
                    f"CAN error count changed by {current_error_count}."
                )
            records.append(
                {
                    "schema_version": 1,
                    "record_type": "piper_commissioning",
                    "adapter_serial": adapter_serial,
                    "sequence": len(records),
                    "wall_time_utc": datetime.now(timezone.utc).isoformat(),
                    "operator": operator,
                    "commissioning_baseline_degrees": session_start.copy(),
                    "joint": joint_index + 1,
                    "requested_delta_degrees": requested_delta,
                    "measured_delta_degrees": measured_delta,
                    "other_joint_max_abs_delta_degrees": other_drift,
                    "motion_speed_percent": motion_speed_percent,
                    "can_error_count": 0,
                    "emergency_stop_verified": False,
                }
            )
            return target

        def return_to_baseline() -> None:
            drive_to_target(
                interface,
                session_start,
                timeout_s=return_timeout_s,
                command_hz=command_hz,
                motion_speed_percent=motion_speed_percent,
                monotonic=monotonic,
                sleep=sleep,
            )

        if manual_step:
            remaining = {
                f"{joint}{sign}"
                for joint in range(1, 7)
                for sign in ("-", "+")
            }
            while remaining:
                completed = 12 - len(remaining)
                command = wait_for_manual_command(
                    interface,
                    session_start,
                    prompt=(
                        f"BASELINE ({completed}/12 complete). Enter one remaining test "
                        f"{', '.join(sorted(remaining))}; s=status, q=abort:"
                    ),
                    allowed_commands=remaining,
                    motion_speed_percent=motion_speed_percent,
                    command_hz=command_hz,
                    can_error_reader=can_error_reader,
                    error_count_before=error_count_before,
                    command_reader=command_reader,
                    monotonic=monotonic,
                    sleep=sleep,
                )
                joint_index = int(command[0]) - 1
                requested_delta = (
                    COMMISSIONING_DELTA_DEG
                    if command[1] == "+"
                    else -COMMISSIONING_DELTA_DEG
                )
                target = run_one_test(joint_index, requested_delta)
                wait_for_manual_command(
                    interface,
                    target,
                    prompt=(
                        f"Holding J{joint_index + 1} {requested_delta:+.1f}°. "
                        "Observe the arm, then enter b to return; s=status, q=abort:"
                    ),
                    allowed_commands={"b"},
                    motion_speed_percent=motion_speed_percent,
                    command_hz=command_hz,
                    can_error_reader=can_error_reader,
                    error_count_before=error_count_before,
                    command_reader=command_reader,
                    monotonic=monotonic,
                    sleep=sleep,
                )
                return_to_baseline()
                remaining.remove(command)
        else:
            for joint_index in range(6):
                for requested_delta in (
                    -COMMISSIONING_DELTA_DEG,
                    COMMISSIONING_DELTA_DEG,
                ):
                    run_one_test(joint_index, requested_delta)
                    return_to_baseline()
                    if len(records) < 12:
                        sleep(inter_test_pause_s)
    except BaseException as exc:
        failure = exc
    finally:
        if arm_attempted and not stop_sent:
            try:
                _send_emergency_stop_once(interface)
                stop_sent = True
            except BaseException as exc:
                stop_failure = exc

    if failure is not None:
        if stop_failure is not None:
            raise CommissioningError(
                f"Commissioning failed ({failure}) and EmergencyStop(1) also failed "
                f"({stop_failure})."
            ) from failure
        raise failure
    if stop_failure is not None or not stop_sent:
        raise CommissioningError(
            "Commissioning completed but EmergencyStop(1) failed."
        ) from stop_failure
    verify_emergency_stop_feedback(interface, monotonic=monotonic, sleep=sleep)
    if len(records) != 12:
        raise CommissioningError(f"Expected 12 commissioning records, got {len(records)}.")
    for record in records:
        record["emergency_stop_verified"] = True
    return records


def validate_and_publish(
    records: list[dict[str, Any]],
    output: Path,
    *,
    expected_adapter_serial: str,
) -> None:
    output = output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite commissioning evidence: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.stem}.", suffix=".pending.jsonl", dir=output.parent
    )
    os.close(descriptor)
    temporary_path = Path(temporary_name)
    with temporary_path.open("w", encoding="utf-8") as stream:
        for record in records:
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    try:
        _validate_commissioning(
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


def main() -> int:
    args = parse_args()
    _validate_cli(args)
    output = args.output.expanduser().resolve()

    actual_serial = read_usb_serial_for_network_interface(args.can_interface)
    if actual_serial != args.expected_adapter_serial:
        raise CommissioningError(
            f"{args.can_interface} uses adapter {actual_serial!r}, expected "
            f"{args.expected_adapter_serial!r}."
        )
    baseline = json.loads(args.safety_baseline.expanduser().read_text(encoding="utf-8"))
    if baseline.get("schema_version") != 1:
        raise CommissioningError("Unsupported safety baseline schema.")
    can_before = read_can_health(args.can_interface)
    ensure_external_control_quiet(
        args.can_interface,
        args.external_control_quiet_s,
    )

    if args.preflight_only:
        interface: Any | None = None
        joints: list[float] | None = None
        preflight_failure: Exception | None = None
        try:
            interface = C_PiperInterface_V2(
                args.can_interface,
                judge_flag=True,
                can_auto_init=True,
                start_sdk_joint_limit=True,
                start_sdk_gripper_limit=True,
            )
            interface.ConnectPort(piper_init=False, start_thread=True)
            joints = wait_for_ready_feedback(interface)
            validate_commissioning_pose(
                joints,
                baseline,
                require_training_envelope=(
                    not args.manual_step and not args.current_pose_deployment_preflight
                ),
                require_bidirectional_margin=(
                    not args.manual_step
                    and not args.deployment_preflight
                    and not args.current_pose_deployment_preflight
                ),
            )
        except Exception as exc:
            preflight_failure = exc
        finally:
            if interface is not None and interface.get_connect_status():
                interface.DisconnectPort()

        can_after = read_can_health(args.can_interface)
        error_delta = can_error_delta(can_before, can_after)
        tx_before = int(can_before["tx_packets"])
        tx_after = int(can_after["tx_packets"])
        rx_before = int(can_before["rx_packets"])
        rx_after = int(can_after["rx_packets"])
        transport_failure: str | None = None
        if tx_after != tx_before:
            transport_failure = (
                f"host CAN TX changed from {tx_before} to {tx_after}"
            )
        elif error_delta != 0:
            transport_failure = f"CAN errors changed by {error_delta}"
        elif rx_after <= rx_before:
            transport_failure = "CAN RX did not increase"

        report = {
            "mode": (
                "receive_only_manual_jog_preflight"
                if args.manual_step
                else (
                    "receive_only_current_pose_deployment_preflight"
                    if args.current_pose_deployment_preflight
                    else (
                        "receive_only_deployment_preflight"
                        if args.deployment_preflight
                        else "receive_only_preflight"
                    )
                )
            ),
            "passed": preflight_failure is None and transport_failure is None,
            "adapter_serial": actual_serial,
            "joint_positions_degrees": joints,
            "feedback_and_pose_error": (
                None if preflight_failure is None else str(preflight_failure)
            ),
            "transport_error": transport_failure,
            "can": {
                "tx_before": tx_before,
                "tx_after": tx_after,
                "rx_before": rx_before,
                "rx_after": rx_after,
                "error_delta": error_delta,
            },
        }
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        if transport_failure is not None:
            raise CommissioningError(
                f"Receive-only preflight violated a CAN gate: {transport_failure}."
            )
        if preflight_failure is not None:
            raise CommissioningError(
                "Receive-only transport checks passed with zero host CAN TX, but the "
                f"feedback/pose gate failed: {preflight_failure}"
            ) from preflight_failure
        preflight_label = (
            "current-pose deployment"
            if args.current_pose_deployment_preflight
            else ("deployment" if args.deployment_preflight else "commissioning")
        )
        print(
            f"PASS: receive-only {preflight_label} preflight completed with zero host CAN TX."
        )
        return 0

    if output.exists():
        raise FileExistsError(f"Refusing to overwrite commissioning evidence: {output}")
    if args.manual_step and not sys.stdin.isatty():
        raise CommissioningError("--manual-step requires an interactive terminal (TTY).")

    print("WARNING: this model-free commissioning will move the follower by +/-0.5 degree.")
    if args.manual_step:
        print(
            "Manual setup may jog each joint in 0.5 degree increments before `c` "
            f"locks the baseline (maximum {args.max_jog_displacement_deg:.1f} "
            "degrees per joint from session start)."
        )
    print("The master must be powered off or physically isolated from the shared CAN bus.")
    print("The operator must hold a tested physical emergency stop throughout the session.")
    typed = input(f"Type {START_CONFIRMATION} to continue: ").strip()
    if typed != START_CONFIRMATION:
        raise CommissioningError("Commissioning confirmation was not accepted; no command sent.")
    # Close the human-confirmation gap: external control must still be quiet immediately
    # before the official SDK interface is allowed to create a transmit path.
    ensure_external_control_quiet(
        args.can_interface,
        args.external_control_quiet_s,
    )

    interface: Any | None = None
    records: list[dict[str, Any]] | None = None
    try:
        interface = C_PiperInterface_V2(
            args.can_interface,
            judge_flag=True,
            can_auto_init=True,
            start_sdk_joint_limit=True,
            start_sdk_gripper_limit=True,
        )
        interface.ConnectPort(piper_init=False, start_thread=True)

        def current_can_error_total() -> int:
            return can_error_delta(can_before, read_can_health(args.can_interface))

        records = execute_commissioning(
            interface,
            baseline=baseline,
            adapter_serial=args.expected_adapter_serial,
            operator=args.operator,
            motion_speed_percent=args.motion_speed_percent,
            command_hz=args.command_hz,
            enable_timeout_s=args.enable_timeout_s,
            baseline_hold_s=args.baseline_hold_s,
            max_jog_displacement_deg=args.max_jog_displacement_deg,
            motion_timeout_s=args.motion_timeout_s,
            return_timeout_s=args.return_timeout_s,
            inter_test_pause_s=args.inter_test_pause_s,
            can_error_reader=current_can_error_total,
            manual_step=args.manual_step,
        )
    finally:
        if interface is not None and interface.get_connect_status():
            interface.DisconnectPort()

    can_after = read_can_health(args.can_interface)
    error_delta = can_error_delta(can_before, can_after)
    if error_delta != 0:
        raise CommissioningError(f"CAN errors changed by {error_delta}; evidence not published.")
    if int(can_after["rx_packets"]) <= int(can_before["rx_packets"]):
        raise CommissioningError("CAN RX did not increase; evidence not published.")
    if records is None:
        raise CommissioningError("Commissioning produced no evidence.")
    validate_and_publish(
        records,
        output,
        expected_adapter_serial=args.expected_adapter_serial,
    )
    print(f"PASS: wrote 12 fixed commissioning records to {output}")
    print("Emergency stop remains latched. Do not automatically resume, home, or reset the arm.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print(
            "ERROR: Operator interrupted commissioning; the one-shot emergency-stop "
            "path was requested if arming had started.",
            file=sys.stderr,
        )
        raise SystemExit(130) from None
    except (CommissioningError, FileExistsError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from None
