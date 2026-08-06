#!/usr/bin/env python3

"""Show read-only follower error relative to one versioned shadow-QA pose."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from lerobot_robot_piper import PiperRobot, PiperRobotConfig


PROJECT_ROOT = Path(__file__).resolve().parents[1]
BASELINE_PATH = PROJECT_ROOT / "config" / "piper_safety_baseline_v1.json"
FEATURES = tuple([f"joint_{index}.pos" for index in range(1, 7)] + ["gripper.pos"])
TX_PATH = Path("/sys/class/net/can0/statistics/tx_packets")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-label", choices=("center", "edge_a", "edge_b"), required=True)
    parser.add_argument("--once", action="store_true", help="Read once instead of updating until Ctrl+C.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    baseline = json.loads(BASELINE_PATH.read_text(encoding="utf-8"))
    target = np.asarray(baseline["qa_poses"][args.pose_label]["state"], dtype=np.float64)
    tolerance = np.asarray(baseline["initial_state"]["tolerance"], dtype=np.float64)
    robot = PiperRobot(
        PiperRobotConfig(
            can_interface="can0",
            expected_adapter_serial="002900225547571120343930",
        )
    )
    tx_before = int(TX_PATH.read_text(encoding="utf-8"))
    passed = False
    try:
        robot.connect(calibrate=False)
        print(f"Target {args.pose_label}: {target.tolist()}")
        print("Move only through the official master/follower teleoperation. Ctrl+C exits.")
        while True:
            observation = robot.get_observation()
            actual = np.asarray([observation[feature] for feature in FEATURES], dtype=np.float64)
            delta = actual - target
            passed = bool((np.abs(delta) <= tolerance).all())
            print(
                f"{'READY' if passed else 'ALIGN'} actual={np.round(actual, 3).tolist()} "
                f"delta={np.round(delta, 3).tolist()}",
                end="\n" if args.once else "\r",
                flush=True,
            )
            if args.once:
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if robot.is_connected:
            robot.disconnect()
        if not args.once:
            print()
    tx_after = int(TX_PATH.read_text(encoding="utf-8"))
    if tx_after != tx_before:
        print(f"FAIL: host CAN TX changed from {tx_before} to {tx_after}.")
        return 1
    print(f"Host CAN TX stayed at {tx_after}. Last pose result: {'READY' if passed else 'ALIGN'}.")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
