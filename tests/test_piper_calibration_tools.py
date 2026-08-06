import ast
import importlib.util
import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
BASELINE = json.loads(
    (ROOT / "config" / "piper_safety_baseline_v1.json").read_text(encoding="utf-8")
)
ADAPTER_SERIAL = "002900225547571120343930"


def load_script(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


passive = load_script(
    "capture_piper_passive_mapping",
    ROOT / "scripts" / "capture_piper_passive_mapping.py",
)
commission = load_script(
    "commission_piper_calibration",
    ROOT / "scripts" / "commission_piper_calibration.py",
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.0, seconds)


class FakePassiveTeleoperator:
    def __init__(self, *, fallback: bool = False) -> None:
        self.sample = 0
        self.last_action = None
        self.fallback = fallback

    def get_action(self):
        offset = self.sample - 9.5
        action = {
            "joint_1.pos": offset,
            "joint_2.pos": 30.0 + offset,
            "joint_3.pos": -50.0 + offset,
            "joint_4.pos": offset,
            "joint_5.pos": offset,
            "joint_6.pos": offset,
            "gripper.pos": self.sample * (105.4 / 19.0),
        }
        self.last_action = action
        self.sample += 1
        return action

    def get_health_stats(self):
        return {
            "action_source": "follower_feedback" if self.fallback else "master_target",
            "joint_target_received": not self.fallback,
            "gripper_target_received": not self.fallback,
            "error_frames": 0,
        }


class FakeStablePassiveTeleoperator(FakePassiveTeleoperator):
    def __init__(self) -> None:
        super().__init__()
        self.call_count = 0

    def get_action(self):
        self.sample = min(self.call_count // 10, 19)
        self.call_count += 1
        return super().get_action()


class FakeDelayedTargetTeleoperator(FakePassiveTeleoperator):
    def __init__(self, ready_after: int) -> None:
        super().__init__()
        self.ready_after = ready_after
        self.health_reads = 0

    def get_health_stats(self):
        self.health_reads += 1
        ready = self.health_reads >= self.ready_after
        return {
            "action_source": "master_target" if ready else "follower_feedback",
            "joint_target_received": ready,
            "gripper_target_received": ready,
            "error_frames": 0,
        }


class FakePassiveRobot:
    def __init__(self, teleoperator: FakePassiveTeleoperator) -> None:
        self.teleoperator = teleoperator

    def get_observation(self):
        master = self.teleoperator.last_action
        assert master is not None
        result = {}
        for index in range(1, 7):
            value = master[f"joint_{index}.pos"]
            result[f"joint_{index}.pos"] = value * (
                1.0 + (index - 1) * 0.002
            ) + (index - 1) * 0.05
        result["gripper.pos"] = min(max(master["gripper.pos"], 0.0), 70.0)
        return result


class FakeCommissioningInterface:
    def __init__(
        self,
        *,
        drift_other_joint: bool = False,
        enable_after: int = 1,
        initial_joints: list[float] | None = None,
        arm_settle_joint_5_offset: float = 0.0,
    ) -> None:
        self.joints = (
            [0.457, 1.0, -1.0, -2.275, 15.681, 1.296]
            if initial_joints is None
            else initial_joints.copy()
        )
        self.baseline = self.joints.copy()
        self.status_code = 0
        self.connected = True
        self.calls = []
        self.drift_other_joint = drift_other_joint
        self.enable_after = enable_after
        self.enable_calls = 0
        self.arm_settle_joint_5_offset = arm_settle_joint_5_offset
        self.arm_settle_rebased = False

    def get_connect_status(self):
        return self.connected

    def isOk(self):
        return True

    def GetArmStatus(self):
        return SimpleNamespace(
            time_stamp=time.time(),
            Hz=200.0,
            arm_status=SimpleNamespace(arm_status=self.status_code, err_code=0),
        )

    def GetArmJointMsgs(self):
        state = SimpleNamespace(
            **{
                f"joint_{index}": int(round(value * 1000))
                for index, value in enumerate(self.joints, 1)
            }
        )
        return SimpleNamespace(time_stamp=time.time(), Hz=200.0, joint_state=state)

    def GetArmGripperMsgs(self):
        foc_status = SimpleNamespace(
            voltage_too_low=False,
            motor_overheating=False,
            driver_overcurrent=False,
            driver_overheating=False,
            sensor_status=False,
            driver_error_status=False,
        )
        return SimpleNamespace(
            time_stamp=time.time(),
            Hz=200.0,
            gripper_state=SimpleNamespace(foc_status=foc_status),
        )

    def EnablePiper(self):
        self.calls.append(("EnablePiper", ()))
        self.enable_calls += 1
        return self.enable_calls >= self.enable_after

    def ModeCtrl(self, *args):
        self.calls.append(("ModeCtrl", args))

    def JointCtrl(self, *args):
        self.calls.append(("JointCtrl", args))
        self.joints = [float(value) * 0.001 for value in args]
        if self.arm_settle_joint_5_offset and not self.arm_settle_rebased:
            if abs(self.joints[4] - self.baseline[4]) <= 0.001:
                self.joints[4] += self.arm_settle_joint_5_offset
            else:
                self.arm_settle_rebased = True
        moving = any(
            abs(value - origin) > 0.25
            for value, origin in zip(self.joints, self.baseline, strict=True)
        )
        if self.drift_other_joint and moving:
            self.joints[5] += 1.0

    def EmergencyStop(self, *args):
        self.calls.append(("EmergencyStop", args))
        self.status_code = 1


class PiperCalibrationToolsTest(unittest.TestCase):
    def test_commissioning_source_has_fixed_sdk_control_allowlist(self) -> None:
        path = ROOT / "scripts" / "commission_piper_calibration.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        control_names = {
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr
            in {
                "EnableArm",
                "EnablePiper",
                "MotionCtrl_2",
                "ModeCtrl",
                "JointCtrl",
                "GripperCtrl",
                "EmergencyStop",
                "ResetPiper",
                "MasterSlaveConfig",
            }
        }
        self.assertEqual(
            control_names,
            {"EnablePiper", "ModeCtrl", "JointCtrl", "EmergencyStop"},
        )
        source = path.read_text(encoding="utf-8")
        self.assertNotIn("EmergencyStop(2)", source)
        self.assertNotIn("ResetPiper(", source)

    def test_passive_default_stability_gate_accepts_twenty_distinct_held_poses(self) -> None:
        clock = FakeClock()
        teleoperator = FakeStablePassiveTeleoperator()
        records = passive.capture_records(
            FakePassiveRobot(teleoperator),
            teleoperator,
            adapter_serial=ADAPTER_SERIAL,
            duration_s=220.0,
            fps=1.0,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
        self.assertEqual(len(records), 20)

    def test_passive_waits_for_dynamic_master_target_burst(self) -> None:
        clock = FakeClock()
        teleoperator = FakeDelayedTargetTeleoperator(ready_after=4)
        passive.wait_for_master_targets(
            teleoperator,
            timeout_s=1.0,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
        self.assertEqual(teleoperator.health_reads, 4)

        never_ready = FakeDelayedTargetTeleoperator(ready_after=100)
        with self.assertRaisesRegex(
            passive.PassiveMappingCaptureError, "physical master is moved"
        ):
            passive.wait_for_master_targets(
                never_ready,
                timeout_s=0.2,
                monotonic=clock.monotonic,
                sleep=clock.sleep,
            )

    def test_passive_capture_and_exact_generator_validation(self) -> None:
        clock = FakeClock()
        teleoperator = FakePassiveTeleoperator()
        robot = FakePassiveRobot(teleoperator)
        records = passive.capture_records(
            robot,
            teleoperator,
            adapter_serial=ADAPTER_SERIAL,
            duration_s=20.0,
            fps=1.0,
            stable_samples=1,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
        self.assertEqual(len(records), 20)
        self.assertEqual(records[0]["record_type"], "piper_passive_mapping")
        self.assertEqual(records[0]["capture_mode"], "read_only")

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "passive.jsonl"
            report = passive.validate_and_publish(
                records,
                output,
                expected_adapter_serial=ADAPTER_SERIAL,
            )
            self.assertEqual(report["samples"], 20)
            self.assertEqual(len(output.read_text(encoding="utf-8").splitlines()), 20)
            with self.assertRaises(FileExistsError):
                passive.validate_and_publish(
                    records,
                    output,
                    expected_adapter_serial=ADAPTER_SERIAL,
                )

    def test_passive_protocol_mapping_keeps_identity_and_gripper_saturation(self) -> None:
        records = []
        for sample_index in range(20):
            offset = sample_index - 9.5
            master = [
                offset,
                30.0 + offset,
                -50.0 + offset,
                offset,
                offset,
                offset,
                sample_index * (105.4 / 19.0),
            ]
            follower = master[:6]
            follower[2] += 0.75 if sample_index % 2 else -0.75
            follower.append(min(max(master[6], 0.0), 70.0))
            records.append(
                {
                    "schema_version": 1,
                    "record_type": "piper_passive_mapping",
                    "capture_mode": "read_only",
                    "adapter_serial": ADAPTER_SERIAL,
                    "master": master,
                    "follower": follower,
                }
            )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "protocol-evidence.jsonl"
            path.write_text(
                "".join(json.dumps(record) + "\n" for record in records),
                encoding="utf-8",
            )
            _, joint_mappings, gripper_mapping = passive._validate_passive_mapping(
                path,
                expected_adapter_serial=ADAPTER_SERIAL,
            )
        self.assertEqual(joint_mappings[2]["scale"], 1.0)
        self.assertEqual(joint_mappings[2]["offset"], 0.0)
        self.assertAlmostEqual(joint_mappings[2]["max_error"], 0.75)
        self.assertEqual(gripper_mapping["max_error"], 0.0)

    def test_passive_capture_rejects_follower_feedback_fallback(self) -> None:
        clock = FakeClock()
        teleoperator = FakePassiveTeleoperator(fallback=True)
        with self.assertRaisesRegex(
            passive.PassiveMappingCaptureError, "Master target frames"
        ):
            passive.capture_records(
                FakePassiveRobot(teleoperator),
                teleoperator,
                adapter_serial=ADAPTER_SERIAL,
                duration_s=20.0,
                fps=1.0,
                stable_samples=1,
                monotonic=clock.monotonic,
                sleep=clock.sleep,
            )

    def test_commissioning_executes_exact_plan_and_latches_one_stop(self) -> None:
        clock = FakeClock()
        interface = FakeCommissioningInterface()
        records = commission.execute_commissioning(
            interface,
            baseline=BASELINE,
            adapter_serial=ADAPTER_SERIAL,
            operator="test-operator",
            motion_speed_percent=5,
            command_hz=20.0,
            motion_timeout_s=3.0,
            return_timeout_s=3.0,
            inter_test_pause_s=0.5,
            can_error_reader=lambda: 0,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
        self.assertEqual(len(records), 12)
        self.assertEqual(
            {(record["joint"], record["requested_delta_degrees"]) for record in records},
            {(joint, delta) for joint in range(1, 7) for delta in (-0.5, 0.5)},
        )
        self.assertTrue(all(record["emergency_stop_verified"] for record in records))
        self.assertTrue(all(record["can_error_count"] == 0 for record in records))
        names = [name for name, _ in interface.calls]
        self.assertEqual(names[:3], ["EnablePiper", "ModeCtrl", "JointCtrl"])
        self.assertEqual(names.count("EnablePiper"), 1)
        self.assertGreater(names.count("ModeCtrl"), 12)
        self.assertEqual(names.count("ModeCtrl"), names.count("JointCtrl"))
        self.assertEqual(names.count("EmergencyStop"), 1)
        self.assertNotIn(("EmergencyStop", (2,)), interface.calls)
        self.assertEqual(interface.calls[-1], ("EmergencyStop", (1,)))

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "commissioning.jsonl"
            commission.validate_and_publish(
                records,
                output,
                expected_adapter_serial=ADAPTER_SERIAL,
            )
            self.assertEqual(len(output.read_text(encoding="utf-8").splitlines()), 12)

    def test_commissioning_receive_only_preflight_makes_no_control_calls(self) -> None:
        clock = FakeClock()
        interface = FakeCommissioningInterface()
        joints = commission.receive_only_preflight(
            interface,
            BASELINE,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
        for actual, expected in zip(joints, interface.joints, strict=True):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertEqual(interface.calls, [])

    def test_commissioning_fault_still_sends_only_one_emergency_stop(self) -> None:
        clock = FakeClock()
        interface = FakeCommissioningInterface(drift_other_joint=True)
        with self.assertRaisesRegex(commission.CommissioningError, "drifted"):
            commission.execute_commissioning(
                interface,
                baseline=BASELINE,
                adapter_serial=ADAPTER_SERIAL,
                operator="test-operator",
                motion_speed_percent=5,
                command_hz=20.0,
                motion_timeout_s=3.0,
                return_timeout_s=3.0,
                inter_test_pause_s=0.5,
                can_error_reader=lambda: 0,
                monotonic=clock.monotonic,
                sleep=clock.sleep,
            )
        emergency_calls = [call for call in interface.calls if call[0] == "EmergencyStop"]
        self.assertEqual(emergency_calls, [("EmergencyStop", (1,))])

    def test_commissioning_waits_for_all_motor_enable_feedback(self) -> None:
        clock = FakeClock()
        interface = FakeCommissioningInterface(enable_after=4)
        commission.wait_until_enabled(
            interface,
            timeout_s=1.0,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
        self.assertEqual(interface.enable_calls, 4)

    def test_manual_commissioning_accepts_only_fixed_steps_and_return(self) -> None:
        clock = FakeClock()
        interface = FakeCommissioningInterface()
        commands = iter(
            ["c"]
            + [
                command
                for joint in range(1, 7)
                for command in (f"{joint}-", "b", f"{joint}+", "b")
            ]
        )
        records = commission.execute_commissioning(
            interface,
            baseline=BASELINE,
            adapter_serial=ADAPTER_SERIAL,
            operator="manual-test-operator",
            motion_speed_percent=5,
            command_hz=20.0,
            motion_timeout_s=3.0,
            return_timeout_s=3.0,
            inter_test_pause_s=0.5,
            can_error_reader=lambda: 0,
            manual_step=True,
            command_reader=lambda: next(commands),
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
        self.assertEqual(len(records), 12)
        self.assertEqual(
            {(record["joint"], record["requested_delta_degrees"]) for record in records},
            {(joint, delta) for joint in range(1, 7) for delta in (-0.5, 0.5)},
        )
        self.assertEqual(
            [call for call in interface.calls if call[0] == "EmergencyStop"],
            [("EmergencyStop", (1,))],
        )

    def test_manual_commissioning_can_jog_from_non_training_pose_before_lock(self) -> None:
        clock = FakeClock()
        start = [25.0, 0.0, 0.0, 30.0, -25.0, 40.0]
        interface = FakeCommissioningInterface(initial_joints=start)
        commands = iter(
            ["2-", "2+", "2+", "3-", "3-", "c"]
            + [
                command
                for joint in range(1, 7)
                for command in (f"{joint}-", "b", f"{joint}+", "b")
            ]
        )
        records = commission.execute_commissioning(
            interface,
            baseline=BASELINE,
            adapter_serial=ADAPTER_SERIAL,
            operator="manual-jog-test-operator",
            motion_speed_percent=5,
            command_hz=20.0,
            motion_timeout_s=3.0,
            return_timeout_s=3.0,
            inter_test_pause_s=0.5,
            can_error_reader=lambda: 0,
            manual_step=True,
            command_reader=lambda: next(commands),
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
        self.assertEqual(len(records), 12)
        self.assertTrue(all(record["emergency_stop_verified"] for record in records))
        # The fixed evidence baseline was locked outside the training start envelope.
        self.assertEqual(
            records[0]["commissioning_baseline_degrees"],
            [25.0, 1.0, -1.0, 30.0, -25.0, 40.0],
        )

    def test_arm_enable_settling_is_stabilized_and_rebased_before_tests(self) -> None:
        clock = FakeClock()
        interface = FakeCommissioningInterface(arm_settle_joint_5_offset=0.532)
        commands = iter(
            ["c"]
            + [
                command
                for joint in range(1, 7)
                for command in (f"{joint}-", "b", f"{joint}+", "b")
            ]
        )
        records = commission.execute_commissioning(
            interface,
            baseline=BASELINE,
            adapter_serial=ADAPTER_SERIAL,
            operator="settling-regression-operator",
            motion_speed_percent=5,
            command_hz=20.0,
            motion_timeout_s=3.0,
            return_timeout_s=3.0,
            inter_test_pause_s=0.5,
            can_error_reader=lambda: 0,
            manual_step=True,
            command_reader=lambda: next(commands),
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
        self.assertEqual(len(records), 12)
        self.assertAlmostEqual(
            records[0]["commissioning_baseline_degrees"][4],
            16.213,
            places=3,
        )
        self.assertTrue(interface.arm_settle_rebased)

    def test_commissioning_pose_requires_bidirectional_physical_margin(self) -> None:
        center = BASELINE["qa_poses"]["center"]["state"][:6]
        with self.assertRaisesRegex(commission.CommissioningError, "physical margin"):
            commission.validate_commissioning_pose(center, BASELINE)

    def test_deployment_pose_keeps_training_gate_without_commissioning_margin(self) -> None:
        center = BASELINE["qa_poses"]["center"]["state"][:6]
        # joint_2 is close to its zero-degree SDK limit and cannot support a
        # negative 0.5-degree commissioning test, but it is a real training
        # start and is valid for fail-closed deployment.
        commission.validate_commissioning_pose(
            center,
            BASELINE,
            require_training_envelope=True,
            require_bidirectional_margin=False,
        )

        outside_training_start = center.copy()
        outside_training_start[2] = -10.0
        with self.assertRaisesRegex(commission.CommissioningError, "training start envelope"):
            commission.validate_commissioning_pose(
                outside_training_start,
                BASELINE,
                require_training_envelope=True,
                require_bidirectional_margin=False,
            )

    def test_can_details_parser_requires_error_active_one_megabit(self) -> None:
        details = """
3: can0: <NOARP,UP,LOWER_UP> mtu 16 state UP
    can state ERROR-ACTIVE restart-ms 0
      bitrate 1000000 sample-point 0.750
      re-started bus-errors arbit-lost error-warn error-pass bus-off
      0          0          0          0          0          0
"""
        health = commission.parse_can_details(details)
        self.assertEqual(health["state"], "ERROR-ACTIVE")
        self.assertEqual(health["bus-off"], 0)
        with self.assertRaisesRegex(commission.CommissioningError, "ERROR-ACTIVE"):
            commission.parse_can_details(details.replace("ERROR-ACTIVE", "ERROR-PASSIVE"))


if __name__ == "__main__":
    unittest.main()
