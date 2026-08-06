#!/usr/bin/env python3

"""Explicitly recover Piper from the software emergency stop between active runs."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Any

from piper_sdk import C_PiperInterface_V2

from commission_piper_calibration import (
    CommissioningError,
    _validate_wrapper,
    can_error_delta,
    ensure_external_control_quiet,
    read_can_health,
    read_joint_feedback,
)
from lerobot_robot_piper.robot_piper import read_usb_serial_for_network_interface


DEFAULT_ADAPTER_SERIAL = "002900225547571120343930"
CONFIRMATION = "RECOVER_PIPER_FROM_SOFTWARE_ESTOP"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument("--expected-adapter-serial", default=DEFAULT_ADAPTER_SERIAL)
    parser.add_argument("--external-control-quiet-s", type=float, default=2.0)
    parser.add_argument("--recovery-timeout-s", type=float, default=3.0)
    return parser.parse_args()


def read_snapshot(interface: Any) -> dict[str, Any]:
    if not interface.get_connect_status() or not interface.isOk():
        raise CommissioningError("Piper SDK connection or CAN receive thread is unhealthy.")
    status = interface.GetArmStatus()
    joints = interface.GetArmJointMsgs()
    gripper = interface.GetArmGripperMsgs()
    now = time.time()
    _validate_wrapper("arm status", status, wall_time=now)
    _validate_wrapper("joint", joints, wall_time=now)
    _validate_wrapper("gripper", gripper, wall_time=now)
    arm = status.arm_status
    return {
        "arm_status": int(arm.arm_status),
        "arm_status_name": str(arm.arm_status),
        "err_code": int(arm.err_code),
        "joint_positions_degrees": read_joint_feedback(
            interface, require_normal_status=False
        ),
    }


def wait_for_snapshot(interface: Any, timeout_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            return read_snapshot(interface)
        except CommissioningError as exc:
            last_error = exc
            time.sleep(0.05)
    raise CommissioningError(
        f"Fresh Piper feedback did not arrive. Last error: {last_error}"
    ) from last_error


def wait_for_normal_status(interface: Any, timeout_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    last_snapshot: dict[str, Any] | None = None
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            last_snapshot = read_snapshot(interface)
            if last_snapshot["arm_status"] == 0 and last_snapshot["err_code"] == 0:
                # Re-run the full normal-status gate before declaring recovery.
                read_joint_feedback(interface, require_normal_status=True)
                return last_snapshot
        except CommissioningError as exc:
            last_error = exc
        time.sleep(0.05)
    raise CommissioningError(
        "Piper did not return to NORMAL after software emergency-stop recovery; "
        f"last_snapshot={last_snapshot}, last_error={last_error}."
    )


def main() -> int:
    args = parse_args()
    if not sys.stdin.isatty():
        raise CommissioningError("Emergency-stop recovery requires an interactive terminal.")
    if (
        not math.isfinite(args.external_control_quiet_s)
        or not 2.0 <= args.external_control_quiet_s <= 10.0
    ):
        raise ValueError("--external-control-quiet-s must be in [2, 10]")
    if (
        not math.isfinite(args.recovery_timeout_s)
        or not 1.0 <= args.recovery_timeout_s <= 10.0
    ):
        raise ValueError("--recovery-timeout-s must be in [1, 10]")

    actual_serial = read_usb_serial_for_network_interface(args.can_interface)
    if actual_serial != args.expected_adapter_serial:
        raise CommissioningError(
            f"{args.can_interface} uses adapter {actual_serial!r}, expected "
            f"{args.expected_adapter_serial!r}."
        )
    before = read_can_health(args.can_interface)
    ensure_external_control_quiet(args.can_interface, args.external_control_quiet_s)

    interface: Any | None = None
    resume_sent = False
    recovered: dict[str, Any] | None = None
    try:
        interface = C_PiperInterface_V2(
            args.can_interface,
            judge_flag=True,
            can_auto_init=True,
            start_sdk_joint_limit=True,
            start_sdk_gripper_limit=True,
        )
        interface.ConnectPort(piper_init=False, start_thread=True)
        initial = wait_for_snapshot(interface, args.recovery_timeout_s)
        if initial["arm_status"] == 0 and initial["err_code"] == 0:
            print("Piper is already NORMAL; no recovery command was sent.")
            return 0
        if initial["arm_status"] != 1 or initial["err_code"] != 0:
            raise CommissioningError(
                "Recovery is allowed only for software EMERGENCY_STOP with err_code=0; "
                f"observed {initial}."
            )

        print(json.dumps({"before_recovery": initial}, ensure_ascii=False, indent=2))
        print("WARNING: recovery clears the software emergency-stop latch.")
        print("Keep the physical emergency stop in hand and keep the workspace clear.")
        typed = input(f"Type {CONFIRMATION} to continue: ").strip()
        if typed != CONFIRMATION:
            raise CommissioningError("Recovery confirmation was not accepted; no command sent.")

        # Re-check the shared bus immediately before creating the single TX event.
        ensure_external_control_quiet(args.can_interface, args.external_control_quiet_s)
        latest = read_snapshot(interface)
        if latest["arm_status"] != 1 or latest["err_code"] != 0:
            raise CommissioningError(f"Piper state changed before recovery: {latest}.")
        interface.EmergencyStop(2)
        resume_sent = True
        recovered = wait_for_normal_status(interface, args.recovery_timeout_s)
    except Exception:
        if resume_sent and interface is not None:
            try:
                interface.EmergencyStop(1)
            except Exception:
                pass
        raise
    finally:
        if interface is not None and interface.get_connect_status():
            interface.DisconnectPort()

    after = read_can_health(args.can_interface)
    error_delta = can_error_delta(before, after)
    if error_delta != 0:
        raise CommissioningError(f"CAN errors changed by {error_delta} during recovery.")
    if int(after["rx_packets"]) <= int(before["rx_packets"]):
        raise CommissioningError("CAN RX did not increase during recovery.")
    if int(after["tx_packets"]) <= int(before["tx_packets"]):
        raise CommissioningError("The recovery command did not increase host CAN TX.")
    print(
        json.dumps(
            {
                "passed": True,
                "adapter_serial": actual_serial,
                "after_recovery": recovered,
                "can": {
                    "tx_before": int(before["tx_packets"]),
                    "tx_after": int(after["tx_packets"]),
                    "rx_before": int(before["rx_packets"]),
                    "rx_after": int(after["rx_packets"]),
                    "error_delta": error_delta,
                },
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (CommissioningError, ValueError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
