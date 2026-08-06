import ast
import hashlib
import importlib.util
import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import can
from lerobot.robots import RobotConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot_robot_piper_active import (
    MOTION_CONFIRMATION,
    ActiveSafetyError,
    PiperActiveRobot,
    PiperActiveRobotConfig,
    PiperSafetyProcessor,
)
from lerobot_robot_piper_active.calibration import (
    CalibrationEvidenceError,
    build_verified_calibration,
)


ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "config" / "piper_safety_baseline_v1.json"
CALIBRATION = ROOT / "config" / "piper_active_calibration_v1.json"
FEATURES = tuple([f"joint_{index}.pos" for index in range(1, 7)] + ["gripper.pos"])
CENTER = [0.457, 0.148, -0.100, -2.275, 15.681, 1.296, 0.100]
ARM_HOLD_CALL_NAMES = ["EnablePiper"] + ["ModeCtrl", "JointCtrl"] * 3


class FakeActiveInterface:
    def __init__(self, fail_on=None, *, arm_settle_joint_5_offset=0.0) -> None:
        now = time.time()
        foc = SimpleNamespace(
            voltage_too_low=False,
            motor_overheating=False,
            driver_overcurrent=False,
            driver_overheating=False,
            sensor_status=False,
            driver_error_status=False,
        )
        self.status = SimpleNamespace(
            time_stamp=now,
            Hz=200.0,
            arm_status=SimpleNamespace(arm_status=0, err_code=0),
        )
        self.joints = SimpleNamespace(
            time_stamp=now,
            Hz=200.0,
            joint_state=SimpleNamespace(
                **{f"joint_{index}": int(round(CENTER[index - 1] * 1000)) for index in range(1, 7)}
            ),
        )
        self.gripper = SimpleNamespace(
            time_stamp=now,
            Hz=200.0,
            gripper_state=SimpleNamespace(
                grippers_angle=int(round(CENTER[6] * 1000)),
                foc_status=foc,
            ),
        )
        self.calls = []
        self.connected = True
        self.fail_on = fail_on
        self.arm_settle_joint_5_offset = arm_settle_joint_5_offset
        self.arm_settle_rebased = False

    def record(self, name, args):
        self.calls.append((name, args))
        if self.fail_on == name:
            raise RuntimeError(f"injected {name} failure")

    def get_connect_status(self):
        return self.connected

    def ConnectPort(self, *args, **kwargs):
        self.calls.append(("ConnectPort", (args, kwargs)))

    def isOk(self):
        return True

    def GetArmStatus(self):
        self.status.time_stamp = time.time()
        return self.status

    def GetArmJointMsgs(self):
        self.joints.time_stamp = time.time()
        return self.joints

    def GetArmGripperMsgs(self):
        self.gripper.time_stamp = time.time()
        return self.gripper

    def EnablePiper(self):
        self.record("EnablePiper", ())
        return True

    def ModeCtrl(self, *args):
        self.record("ModeCtrl", args)

    def JointCtrl(self, *args):
        self.record("JointCtrl", args)
        values = [int(value) for value in args]
        if self.arm_settle_joint_5_offset and not self.arm_settle_rebased:
            original = int(round(CENTER[4] * 1000))
            if abs(values[4] - original) <= 1:
                values[4] += int(round(self.arm_settle_joint_5_offset * 1000))
            else:
                self.arm_settle_rebased = True
        for index, value in enumerate(values, 1):
            setattr(self.joints.joint_state, f"joint_{index}", value)

    def GripperCtrl(self, *args):
        self.record("GripperCtrl", args)

    def EmergencyStop(self, *args):
        self.record("EmergencyStop", args)

    def DisconnectPort(self):
        self.calls.append(("DisconnectPort", ()))
        self.connected = False


class FakeBus:
    def __init__(self, messages):
        self.messages = list(messages)
        self.shutdown_count = 0

    def recv(self, timeout):
        del timeout
        if self.messages:
            return self.messages.pop(0)
        return None

    def shutdown(self):
        self.shutdown_count += 1


class FakeCamera:
    height = 480
    width = 640
    use_depth = False

    def __init__(self):
        self.is_connected = False

    def connect(self):
        self.is_connected = True

    def read_latest(self):
        return None

    def get_health_stats(self):
        return {"latest_frame_age_ms": 1.0}

    def disconnect(self):
        self.is_connected = False


class PiperActivePluginTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory()
        calibration = json.loads(CALIBRATION.read_text(encoding="utf-8"))
        calibration["schema_version"] = 2
        calibration["calibration_version"] = 2
        calibration["verified"] = True
        calibration["sdk_physical_limits"] = {
            "joint_min_degrees": [-150.0, 0.0, -170.0, -100.0, -70.0, -120.0],
            "joint_max_degrees": [150.0, 180.0, 0.0, 100.0, 70.0, 120.0],
            "gripper_min_mm": 0.0,
            "gripper_max_mm": 70.0,
        }
        passive_evidence = Path(self.tempdir.name) / "fake-passive-evidence.jsonl"
        passive_evidence.write_text("fake passive evidence\n", encoding="utf-8")
        commissioning_evidence = Path(self.tempdir.name) / "fake-commissioning-evidence.jsonl"
        commissioning_evidence.write_text("fake commissioning evidence\n", encoding="utf-8")
        calibration["verification"] = {
            "generator": "lerobot_robot_piper_active.calibration",
            "evidence": {
                "passive_mapping": {
                    "path": str(passive_evidence),
                    "sha256": hashlib.sha256(passive_evidence.read_bytes()).hexdigest(),
                },
                "commissioning": {
                    "path": str(commissioning_evidence),
                    "sha256": hashlib.sha256(commissioning_evidence.read_bytes()).hexdigest(),
                },
            },
        }
        self.verified_calibration = Path(self.tempdir.name) / "verified.json"
        self.verified_calibration.write_text(json.dumps(calibration), encoding="utf-8")

    def tearDown(self) -> None:
        self.tempdir.cleanup()

    def active_config(self, **overrides) -> PiperActiveRobotConfig:
        values = {
            "motion_enabled": True,
            "operator_confirmation": MOTION_CONFIRMATION,
            "safety_baseline_path": str(BASELINE),
            "calibration_path": str(self.verified_calibration),
            "arm_hold_s": 0.05,
            "arm_hold_command_hz": 50.0,
        }
        values.update(overrides)
        return PiperActiveRobotConfig(**values)

    def attach_fake_hardware(self, robot, *, fail_on=None):
        interface = FakeActiveInterface(fail_on=fail_on)
        robot._interface = interface
        robot.cameras = {"front": FakeCamera(), "wrist": FakeCamera()}
        for camera in robot.cameras.values():
            camera.connect()
        robot._validate_adapter_identity = lambda: None
        robot._listen_for_external_control = lambda duration_s=None: None
        return interface

    def write_calibration_evidence(self):
        passive_path = Path(self.tempdir.name) / "passive.jsonl"
        passive_records = []
        for sample_index in range(20):
            offset = sample_index - 9.5
            master = [offset, 30.0 + offset, -50.0 + offset, offset, offset, offset]
            master.append(sample_index * (105.4 / 19.0))
            follower = [
                master[joint] * (1.0 + joint * 0.002) + joint * 0.05
                for joint in range(6)
            ]
            follower.append(min(max(master[6], 0.0), 70.0))
            passive_records.append(
                {
                    "schema_version": 1,
                    "record_type": "piper_passive_mapping",
                    "capture_mode": "read_only",
                    "adapter_serial": "002900225547571120343930",
                    "master": master,
                    "follower": follower,
                }
            )
        passive_path.write_text(
            "".join(json.dumps(record) + "\n" for record in passive_records),
            encoding="utf-8",
        )

        commissioning_path = Path(self.tempdir.name) / "commissioning.jsonl"
        commissioning_records = []
        for joint in range(1, 7):
            for requested in (-0.5, 0.5):
                commissioning_records.append(
                    {
                        "schema_version": 1,
                        "record_type": "piper_commissioning",
                        "adapter_serial": "002900225547571120343930",
                        "joint": joint,
                        "requested_delta_degrees": requested,
                        "measured_delta_degrees": requested * 0.98,
                        "other_joint_max_abs_delta_degrees": 0.02,
                        "motion_speed_percent": 5,
                        "can_error_count": 0,
                        "emergency_stop_verified": True,
                    }
                )
        commissioning_path.write_text(
            "".join(json.dumps(record) + "\n" for record in commissioning_records),
            encoding="utf-8",
        )
        return passive_path, commissioning_path

    def test_plugin_registration_and_default_zero_motion(self) -> None:
        register_third_party_plugins()
        config = PiperActiveRobotConfig()
        self.assertEqual(config.type, "piper_active")
        self.assertFalse(config.motion_enabled)
        self.assertIs(RobotConfig.get_choice_class("piper_active"), PiperActiveRobotConfig)
        self.assertIsInstance(make_robot_from_config(config), PiperActiveRobot)

    def test_v1_calibration_blocks_motion_at_config_parse(self) -> None:
        with self.assertRaisesRegex(ValueError, "v2 calibration"):
            PiperActiveRobotConfig(
                motion_enabled=True,
                operator_confirmation=MOTION_CONFIRMATION,
                calibration_path=str(CALIBRATION),
            )

    def test_missing_calibration_blocks_motion(self) -> None:
        with self.assertRaisesRegex(ValueError, "calibration is missing"):
            PiperActiveRobotConfig(
                motion_enabled=True,
                operator_confirmation=MOTION_CONFIRMATION,
                calibration_path=str(Path(self.tempdir.name) / "missing.json"),
            )

    def test_exact_operator_confirmation_is_required(self) -> None:
        with self.assertRaisesRegex(ValueError, "exact operator confirmation"):
            PiperActiveRobotConfig(
                motion_enabled=True,
                operator_confirmation="yes",
                calibration_path=str(self.verified_calibration),
            )

    def test_new_active_limits_are_fail_closed_at_config_parse(self) -> None:
        self.assertEqual(PiperActiveRobotConfig().safety_profile, "strict")
        self.assertFalse(PiperActiveRobotConfig().arm_on_first_action)
        self.assertEqual(
            PiperActiveRobotConfig().start_pose_mode,
            "training_envelope",
        )
        for overrides, message in (
            ({"safety_profile": "observe"}, "safety_profile"),
            ({"start_pose_mode": "unchecked"}, "start_pose_mode"),
            ({"max_active_actions": -1}, "max_active_actions"),
            ({"max_active_actions": 1.5}, "max_active_actions"),
            ({"max_motion_duration_s": -0.1}, "max_motion_duration_s"),
            ({"max_joint_displacement_deg": 0.0}, "max_joint_displacement_deg"),
            ({"max_gripper_displacement_mm": 0.0}, "max_gripper_displacement_mm"),
            ({"enable_timeout_s": 0.0}, "enable_timeout_s"),
            ({"arm_hold_s": 0.0}, "arm_hold_s"),
            ({"arm_hold_command_hz": 51.0}, "arm_hold_command_hz"),
            ({"arm_hold_tolerance_deg": 0.0}, "arm_hold_tolerance_deg"),
            ({"arm_acquisition_max_drift_deg": 0.1}, "arm_acquisition_max_drift_deg"),
            ({"arm_acquisition_stability_deg": 0.0}, "arm_acquisition_stability_deg"),
        ):
            with self.subTest(overrides=overrides):
                with self.assertRaisesRegex(ValueError, message):
                    PiperActiveRobotConfig(**overrides)

    def test_v2_calibration_is_derived_from_both_evidence_files(self) -> None:
        passive_path, commissioning_path = self.write_calibration_evidence()
        document = build_verified_calibration(
            passive_path,
            commissioning_path,
            expected_adapter_serial="002900225547571120343930",
            can_interface="can0",
            operator="fake-sdk-test",
        )
        self.assertEqual(document["schema_version"], 2)
        self.assertEqual(document["calibration_version"], 2)
        self.assertTrue(document["verified"])
        self.assertEqual(len(document["joints"]), 6)
        self.assertEqual(len(document["gripper_points"]), 3)
        self.assertTrue(
            all(
                mapping["scale"] == 1.0 and mapping["offset_degrees"] == 0.0
                for mapping in document["joints"]
            )
        )
        self.assertEqual(
            document["gripper_points"],
            [
                {"input_mm": -5.0, "output_mm": 0.0},
                {"input_mm": 70.0, "output_mm": 70.0},
                {"input_mm": 120.0, "output_mm": 70.0},
            ],
        )
        self.assertEqual(
            document["verification"]["generator"],
            "lerobot_robot_piper_active.calibration",
        )

        generated = Path(self.tempdir.name) / "generated-v2.json"
        generated.write_text(json.dumps(document), encoding="utf-8")
        config = self.active_config(calibration_path=str(generated))
        safety = PiperActiveRobot(config).safety
        self.assertTrue(safety.calibration_verified)
        high_gripper = dict(zip(FEATURES, CENTER))
        high_gripper["gripper.pos"] = 105.0
        self.assertEqual(safety.prepare(high_gripper)["gripper_units_0_001_mm"], 70000)

    def test_v2_calibration_refuses_incomplete_commissioning(self) -> None:
        passive_path, commissioning_path = self.write_calibration_evidence()
        records = commissioning_path.read_text(encoding="utf-8").splitlines()
        commissioning_path.write_text("\n".join(records[:-1]) + "\n", encoding="utf-8")
        with self.assertRaisesRegex(CalibrationEvidenceError, "missing"):
            build_verified_calibration(
                passive_path,
                commissioning_path,
                expected_adapter_serial="002900225547571120343930",
                can_interface="can0",
                operator="fake-sdk-test",
            )

    def test_v2_calibrated_sdk_target_is_physically_bounded(self) -> None:
        calibration = json.loads(self.verified_calibration.read_text(encoding="utf-8"))
        calibration["joints"][0]["offset_degrees"] = 200.0
        unsafe_path = Path(self.tempdir.name) / "unsafe-mapping-v2.json"
        unsafe_path.write_text(json.dumps(calibration), encoding="utf-8")
        safety = PiperSafetyProcessor(BASELINE, unsafe_path)
        center = dict(zip(FEATURES, CENTER))
        safety.validate_initial_state(center)
        with self.assertRaisesRegex(ActiveSafetyError, "SDK physical limit"):
            safety.prepare(center)

    def test_v2_evidence_tampering_blocks_active_config(self) -> None:
        calibration = json.loads(self.verified_calibration.read_text(encoding="utf-8"))
        evidence_path = Path(
            calibration["verification"]["evidence"]["passive_mapping"]["path"]
        )
        evidence_path.write_text("tampered\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "hash does not match"):
            self.active_config()

    def test_external_master_control_frame_is_rejected(self) -> None:
        robot = PiperActiveRobot(PiperActiveRobotConfig())
        fake_bus = FakeBus([can.Message(arbitration_id=0x155, data=b"\0" * 8)])
        with self.assertRaisesRegex(ActiveSafetyError, "External master command"):
            robot._listen_for_external_control(
                duration_s=0.01,
                bus_factory=lambda **kwargs: fake_bus,
            )
        self.assertEqual(fake_bus.shutdown_count, 1)

    def test_initial_pose_and_task_physical_envelopes(self) -> None:
        safety = PiperSafetyProcessor(BASELINE, CALIBRATION)
        center = dict(zip(FEATURES, CENTER))
        safety.validate_initial_state(center)
        invalid = center.copy()
        invalid["joint_1.pos"] = 151.0
        with self.assertRaisesRegex(ActiveSafetyError, "physical limit"):
            PiperSafetyProcessor(BASELINE, CALIBRATION).validate_initial_state(invalid)
        invalid_action = center.copy()
        invalid_action["joint_4.pos"] = -20.0
        with self.assertRaisesRegex(ActiveSafetyError, "task envelope"):
            safety.prepare(invalid_action)

    def test_current_physical_start_pose_becomes_dynamic_anchor(self) -> None:
        current = dict(zip(FEATURES, CENTER))
        current["joint_5.pos"] = 25.052
        safety = PiperSafetyProcessor(
            BASELINE,
            CALIBRATION,
            profile="micro_observe",
            start_pose_mode="current_physical",
        )
        safety.validate_initial_state(current)
        self.assertEqual(safety.initial_state.tolist(), list(current.values()))
        self.assertEqual(safety.last_safe_action.tolist(), list(current.values()))

        unsafe = current.copy()
        unsafe["joint_2.pos"] = -0.001
        with self.assertRaisesRegex(ActiveSafetyError, "physical limit"):
            safety.validate_initial_state(unsafe)

    def test_micro_observe_warns_but_still_slew_and_displacement_limits(self) -> None:
        center = dict(zip(FEATURES, CENTER))
        safety = PiperSafetyProcessor(
            BASELINE,
            CALIBRATION,
            profile="micro_observe",
        )
        safety.validate_initial_state(center)
        outside_task = center.copy()
        outside_task["joint_4.pos"] = -20.0
        command = safety.prepare(outside_task)
        self.assertEqual(len(command["warnings"]), 2)
        self.assertTrue(command["was_slew_limited"][3])

        physical = center.copy()
        physical["joint_1.pos"] = 151.0
        physical_command = safety.prepare(physical)
        self.assertTrue(physical_command["was_physical_clipped"][0])
        self.assertEqual(physical_command["physical_clipped"][0], 150.0)
        self.assertTrue(
            any("physical limit" in warning for warning in physical_command["warnings"])
        )

        strict_physical = center.copy()
        strict_physical["joint_1.pos"] = 151.0
        with self.assertRaisesRegex(ActiveSafetyError, "physical limit"):
            PiperSafetyProcessor(BASELINE, CALIBRATION).prepare(strict_physical)

        narrow = PiperSafetyProcessor(
            BASELINE,
            CALIBRATION,
            profile="micro_observe",
            max_joint_displacement_deg=0.1,
        )
        narrow.validate_initial_state(center)
        too_far = center.copy()
        too_far["joint_1.pos"] += 0.5
        narrow_command = narrow.prepare(too_far)
        self.assertAlmostEqual(narrow_command["limited"][0], CENTER[0] + 0.1)
        self.assertTrue(narrow_command["was_displacement_clipped"][0])
        self.assertTrue(
            any("displacement window" in warning for warning in narrow_command["warnings"])
        )

        no_window = PiperSafetyProcessor(
            BASELINE,
            CALIBRATION,
            profile="micro_observe",
            max_joint_displacement_deg=0.1,
            enforce_displacement_window=False,
        )
        no_window.validate_initial_state(center)
        unbounded_command = no_window.prepare(too_far)
        self.assertGreater(unbounded_command["limited"][0], CENTER[0] + 0.1)
        self.assertFalse(any(unbounded_command["was_displacement_clipped"]))

    def test_historical_max_latches_before_send_and_p99_slew_limits(self) -> None:
        safety = PiperSafetyProcessor(BASELINE, CALIBRATION)
        center = dict(zip(FEATURES, CENTER))
        safety.validate_initial_state(center)
        hard_jump = center.copy()
        hard_jump["joint_1.pos"] += 3.0
        with self.assertRaisesRegex(ActiveSafetyError, "historical"):
            safety.prepare(hard_jump)

        safety.validate_initial_state(center)
        slew = center.copy()
        slew["joint_1.pos"] += 0.5
        command = safety.prepare(slew)
        baseline = json.loads(BASELINE.read_text(encoding="utf-8"))
        expected = CENTER[0] + baseline["absolute_action_delta"]["p99"][0]
        self.assertAlmostEqual(command["limited"][0], expected)
        self.assertTrue(command["was_slew_limited"][0])

    def test_unit_conversion_and_sdk_call_order(self) -> None:
        robot = PiperActiveRobot(self.active_config())
        interface = self.attach_fake_hardware(robot)
        robot.arm_for_motion()
        action = dict(zip(FEATURES, CENTER))
        robot.send_action(action)
        self.assertEqual(
            [call[0] for call in interface.calls],
            ARM_HOLD_CALL_NAMES + ["ModeCtrl", "JointCtrl", "GripperCtrl"],
        )
        self.assertEqual(interface.calls[-3][1], (1, 1, 10, 0))
        self.assertEqual(
            interface.calls[-2][1],
            tuple(int(round(v * 1000)) for v in CENTER[:6]),
        )
        self.assertEqual(interface.calls[-1][1][1:], (500, 1, 0))
        robot.disconnect()
        self.assertEqual(
            [call[0] for call in interface.calls][-2:],
            ["EmergencyStop", "DisconnectPort"],
        )

    def test_active_command_log_records_motor_angles_and_sdk_units(self) -> None:
        command_log = Path(self.tempdir.name) / "commands.jsonl"
        robot = PiperActiveRobot(
            self.active_config(active_command_log_path=str(command_log))
        )
        interface = self.attach_fake_hardware(robot)
        robot.arm_for_motion()
        robot.send_action(dict(zip(FEATURES, CENTER)))
        robot.disconnect()

        records = [
            json.loads(line)
            for line in command_log.read_text(encoding="utf-8").splitlines()
        ]
        joint_units = [args for name, args in interface.calls if name == "JointCtrl"][-1]
        gripper_units = [args for name, args in interface.calls if name == "GripperCtrl"][-1]
        self.assertEqual(len(records), 1)
        record = records[0]
        self.assertEqual(record["result"], "sent")
        self.assertEqual(record["calibrated_joint_degrees"], CENTER[:6])
        self.assertEqual(
            record["joint_units_0_001_degree"],
            list(joint_units),
        )
        self.assertEqual(
            record["gripper_units_0_001_mm"],
            gripper_units[0],
        )
        self.assertAlmostEqual(
            record["calibrated_gripper_mm"],
            record["gripper_units_0_001_mm"] / 1000.0,
            delta=0.0005,
        )

    def test_active_command_log_records_previous_target_tracking_error(self) -> None:
        command_log = Path(self.tempdir.name) / "tracking-commands.jsonl"
        robot = PiperActiveRobot(
            self.active_config(active_command_log_path=str(command_log))
        )
        self.attach_fake_hardware(robot)
        robot.arm_for_motion()
        robot._last_observation_state = dict(zip(FEATURES, CENTER))
        robot.send_action(dict(zip(FEATURES, CENTER)))
        observed = CENTER.copy()
        observed[0] -= 0.1
        robot._last_observation_state = dict(zip(FEATURES, observed))
        robot.send_action(dict(zip(FEATURES, CENTER)))
        robot.disconnect()

        records = [
            json.loads(line)
            for line in command_log.read_text(encoding="utf-8").splitlines()
        ]
        self.assertIsNone(records[0]["previous_sent_calibrated_target"])
        self.assertEqual(records[1]["feedback_before_command"], observed)
        self.assertAlmostEqual(records[1]["previous_target_tracking_error"][0], 0.1)

    def test_rtc_delay_uses_actual_queue_consumption(self) -> None:
        import torch
        from lerobot.policies.rtc import ActionQueue
        from lerobot.policies.rtc.configuration_rtc import RTCConfig

        queue = ActionQueue(RTCConfig(enabled=True))
        actions = torch.zeros((50, 7), dtype=torch.float32)
        queue.merge(actions, actions, real_delay=60, action_index_before_inference=0)
        self.assertEqual(queue.qsize(), 50)
        for _ in range(20):
            queue.get()
        queue.merge(actions, actions, real_delay=60, action_index_before_inference=0)
        self.assertEqual(queue.qsize(), 30)

    def test_micro_observe_clips_raw_physical_violation_before_sdk(self) -> None:
        command_log = Path(self.tempdir.name) / "clipped-commands.jsonl"
        robot = PiperActiveRobot(
            self.active_config(
                safety_profile="micro_observe",
                start_pose_mode="current_physical",
                active_command_log_path=str(command_log),
            )
        )
        interface = self.attach_fake_hardware(robot)
        robot.arm_for_motion()
        action = dict(zip(FEATURES, CENTER))
        action["joint_2.pos"] = -2.0
        robot.send_action(action)
        robot.disconnect()

        record = json.loads(command_log.read_text(encoding="utf-8"))
        self.assertEqual(record["result"], "sent")
        self.assertTrue(record["was_physical_clipped"][1])
        self.assertEqual(record["physical_clipped_action"][1], 0.0)
        self.assertGreaterEqual(record["limited_action"][1], 0.0)
        joint_units = [args for name, args in interface.calls if name == "JointCtrl"][-1]
        self.assertEqual(joint_units[1], int(round(record["limited_action"][1] * 1000)))

    def test_pre_arm_safety_rejection_is_written_to_command_log(self) -> None:
        command_log = Path(self.tempdir.name) / "rejected-commands.jsonl"
        robot = PiperActiveRobot(
            self.active_config(
                arm_on_first_action=True,
                active_command_log_path=str(command_log),
            )
        )
        interface = self.attach_fake_hardware(robot)
        robot.get_observation = lambda: dict(zip(FEATURES, CENTER))
        action = dict(zip(FEATURES, CENTER))
        action["joint_2.pos"] = -2.0
        with self.assertRaisesRegex(ActiveSafetyError, "physical limit"):
            robot.send_action(action)
        robot.disconnect()

        record = json.loads(command_log.read_text(encoding="utf-8"))
        self.assertEqual(record["result"], "safety_rejected")
        self.assertEqual(record["raw_model_action"][1], -2.0)
        self.assertIn("physical limit", record["safety_error"])
        self.assertNotIn("EnablePiper", [name for name, _ in interface.calls])

    def test_arm_enable_settling_is_rebased_before_policy_actions(self) -> None:
        robot = PiperActiveRobot(self.active_config())
        interface = FakeActiveInterface(arm_settle_joint_5_offset=0.532)
        robot._interface = interface
        robot.cameras = {"front": FakeCamera(), "wrist": FakeCamera()}
        for camera in robot.cameras.values():
            camera.connect()
        robot._validate_adapter_identity = lambda: None
        robot._listen_for_external_control = lambda duration_s=None: None

        robot.arm_for_motion()

        self.assertTrue(robot._armed)
        self.assertFalse(robot._fault_latched)
        self.assertTrue(interface.arm_settle_rebased)
        self.assertAlmostEqual(float(robot.safety.initial_state[4]), 16.213, places=3)
        self.assertEqual(
            [name for name, _ in interface.calls].count("EnablePiper"),
            1,
        )
        robot.disconnect()

    def test_connect_is_passive_and_first_qualified_action_arms(self) -> None:
        robot = PiperActiveRobot(self.active_config(arm_on_first_action=True))
        interface = FakeActiveInterface()
        robot.cameras = {"front": FakeCamera(), "wrist": FakeCamera()}
        robot._validate_adapter_identity = lambda: None
        robot._listen_for_external_control = lambda duration_s=None: None
        with mock.patch(
            "lerobot_robot_piper.robot_piper.C_PiperInterface_V2",
            return_value=interface,
        ):
            robot.connect()
        self.assertEqual([name for name, _ in interface.calls], ["ConnectPort"])

        robot.send_action(dict(zip(FEATURES, CENTER)))
        self.assertEqual(
            [name for name, _ in interface.calls],
            ["ConnectPort"]
            + ARM_HOLD_CALL_NAMES
            + ["ModeCtrl", "JointCtrl", "GripperCtrl"],
        )
        robot.disconnect()

    def test_rejected_first_action_never_arms_and_latches(self) -> None:
        robot = PiperActiveRobot(self.active_config(arm_on_first_action=True))
        interface = self.attach_fake_hardware(robot)
        invalid = dict(zip(FEATURES, CENTER))
        invalid["joint_1.pos"] = 151.0
        with self.assertRaisesRegex(ActiveSafetyError, "physical limit"):
            robot.send_action(invalid)
        self.assertEqual(interface.calls, [])
        self.assertTrue(robot._fault_latched)
        with self.assertRaisesRegex(ActiveSafetyError, "Fault is latched"):
            robot.send_action(dict(zip(FEATURES, CENTER)))
        robot.disconnect()

    def test_bad_schema_and_non_finite_first_actions_never_arm(self) -> None:
        bad_actions = []
        missing = dict(zip(FEATURES, CENTER))
        missing.pop("joint_6.pos")
        bad_actions.append(missing)
        non_finite = dict(zip(FEATURES, CENTER))
        non_finite["joint_2.pos"] = float("nan")
        bad_actions.append(non_finite)
        for bad_action in bad_actions:
            with self.subTest(keys=tuple(bad_action), values=tuple(bad_action.values())):
                robot = PiperActiveRobot(self.active_config(arm_on_first_action=True))
                interface = self.attach_fake_hardware(robot)
                with self.assertRaises(ActiveSafetyError):
                    robot.send_action(bad_action)
                self.assertEqual(interface.calls, [])
                self.assertTrue(robot._fault_latched)
                robot.disconnect()

    def test_action_budget_stops_once_and_suppresses_later_actions(self) -> None:
        robot = PiperActiveRobot(
            self.active_config(arm_on_first_action=True, max_active_actions=2)
        )
        interface = self.attach_fake_hardware(robot)
        action = dict(zip(FEATURES, CENTER))
        robot.send_action(action)
        robot.send_action(action)
        robot.send_action(action)
        robot.send_action(action)
        self.assertEqual(
            [name for name, _ in interface.calls],
            ARM_HOLD_CALL_NAMES
            + [
                "ModeCtrl", "JointCtrl", "GripperCtrl",
                "ModeCtrl", "JointCtrl", "GripperCtrl",
                "EmergencyStop",
            ],
        )
        self.assertEqual(robot._active_action_count, 2)
        self.assertEqual(robot._suppressed_action_count, 2)
        self.assertTrue(robot.rollout_stop_requested)
        self.assertEqual(robot.rollout_stop_reason, "active action budget reached (2)")
        robot.disconnect()
        self.assertEqual(
            [name for name, _ in interface.calls].count("EmergencyStop"),
            1,
        )

    def test_motion_duration_stops_before_another_action(self) -> None:
        robot = PiperActiveRobot(
            self.active_config(arm_on_first_action=True, max_motion_duration_s=0.5)
        )
        interface = self.attach_fake_hardware(robot)
        action = dict(zip(FEATURES, CENTER))
        robot.send_action(action)
        robot._motion_started_monotonic = time.monotonic() - 1.0
        robot.send_action(action)
        self.assertEqual(
            [name for name, _ in interface.calls],
            ARM_HOLD_CALL_NAMES
            + ["ModeCtrl", "JointCtrl", "GripperCtrl", "EmergencyStop"],
        )
        robot.disconnect()

    def test_official_sync_and_rtc_robot_wrapper_delegate_to_send_action(self) -> None:
        from lerobot.rollout.robot_wrapper import ThreadSafeRobot

        robot = PiperActiveRobot(
            self.active_config(arm_on_first_action=True, max_active_actions=1)
        )
        interface = self.attach_fake_hardware(robot)
        wrapper = ThreadSafeRobot(robot)
        wrapper.send_action(dict(zip(FEATURES, CENTER)))
        self.assertTrue(wrapper.rollout_stop_requested)
        self.assertEqual(
            wrapper.rollout_stop_reason,
            "active action budget reached (1)",
        )
        self.assertEqual(
            [name for name, _ in interface.calls],
            ARM_HOLD_CALL_NAMES
            + ["ModeCtrl", "JointCtrl", "GripperCtrl", "EmergencyStop"],
        )
        robot.disconnect()

    def test_base_rollout_exits_before_observing_an_intentional_stop(self) -> None:
        from lerobot.rollout import BaseStrategyConfig
        from lerobot.rollout.strategies.base import BaseStrategy

        get_observation = mock.Mock(
            side_effect=AssertionError("stopped robot must not be observed")
        )
        robot_wrapper = SimpleNamespace(
            rollout_stop_requested=True,
            rollout_stop_reason="active action budget reached (20)",
            get_observation=get_observation,
        )
        engine = SimpleNamespace(resume=mock.Mock())
        interpolator = SimpleNamespace(get_control_interval=lambda fps: 1.0 / fps)
        ctx = SimpleNamespace(
            runtime=SimpleNamespace(
                cfg=SimpleNamespace(fps=10, duration=12.0),
                shutdown_event=SimpleNamespace(is_set=lambda: False),
            ),
            hardware=SimpleNamespace(robot_wrapper=robot_wrapper),
        )
        strategy = BaseStrategy(BaseStrategyConfig())
        strategy._engine = engine
        strategy._interpolator = interpolator

        strategy.run(ctx)

        engine.resume.assert_called_once_with()
        get_observation.assert_not_called()

    def test_official_async_client_delegates_to_send_action(self) -> None:
        if importlib.util.find_spec("grpc") is None:
            self.skipTest("LeRobot async extra is not installed")

        import threading
        from queue import Queue

        import torch
        from lerobot.async_inference.helpers import TimedAction
        from lerobot.async_inference.robot_client import RobotClient

        robot = PiperActiveRobot(
            self.active_config(arm_on_first_action=True, max_active_actions=1)
        )
        interface = self.attach_fake_hardware(robot)
        client = RobotClient.__new__(RobotClient)
        client.robot = robot
        client.action_queue = Queue()
        client.action_queue_lock = threading.Lock()
        client.action_queue_size = []
        client.latest_action_lock = threading.Lock()
        client.latest_action = -1
        client.action_queue.put(
            TimedAction(
                timestamp=time.time(),
                timestep=0,
                action=torch.tensor(CENTER, dtype=torch.float32),
            )
        )
        client.control_loop_action()
        self.assertEqual(
            [name for name, _ in interface.calls],
            ARM_HOLD_CALL_NAMES
            + ["ModeCtrl", "JointCtrl", "GripperCtrl", "EmergencyStop"],
        )
        robot.disconnect()

    def test_missing_training_cameras_blocks_before_enable(self) -> None:
        robot = PiperActiveRobot(self.active_config())
        interface = FakeActiveInterface()
        robot._interface = interface
        robot._validate_adapter_identity = lambda: None
        robot._listen_for_external_control = lambda duration_s=None: None
        with self.assertRaisesRegex(ActiveSafetyError, "front and wrist"):
            robot.arm_for_motion()
        self.assertEqual(interface.calls, [])

    def test_motion_mode_failure_uses_one_emergency_stop_and_latches(self) -> None:
        robot = PiperActiveRobot(self.active_config())
        interface = FakeActiveInterface(fail_on="ModeCtrl")
        robot._interface = interface
        robot.cameras = {"front": FakeCamera(), "wrist": FakeCamera()}
        for camera in robot.cameras.values():
            camera.connect()
        robot._validate_adapter_identity = lambda: None
        robot._listen_for_external_control = lambda duration_s=None: None
        with self.assertRaisesRegex(RuntimeError, "injected"):
            robot.arm_for_motion()
        self.assertEqual(
            [call[0] for call in interface.calls],
            ["EnablePiper", "ModeCtrl", "EmergencyStop"],
        )
        self.assertTrue(robot._fault_latched)
        self.assertFalse(robot.rollout_stop_requested)
        robot.disconnect()

    def test_joint_command_failure_stops_before_gripper_and_latches(self) -> None:
        robot = PiperActiveRobot(self.active_config())
        interface = self.attach_fake_hardware(robot)
        robot.arm_for_motion()
        interface.fail_on = "JointCtrl"
        with self.assertRaisesRegex(RuntimeError, "injected"):
            robot.send_action(dict(zip(FEATURES, CENTER)))
        self.assertEqual(
            [call[0] for call in interface.calls],
            ARM_HOLD_CALL_NAMES + ["ModeCtrl", "JointCtrl", "EmergencyStop"],
        )
        self.assertTrue(robot._fault_latched)
        robot.disconnect()

    def test_default_send_action_never_calls_control_sdk(self) -> None:
        robot = PiperActiveRobot(PiperActiveRobotConfig())
        interface = FakeActiveInterface()
        robot._interface = interface
        robot.send_action(dict(zip(FEATURES, CENTER)))
        self.assertEqual(interface.calls, [])

    def test_watchdog_emergency_stop_is_single_shot_and_fault_latched(self) -> None:
        robot = PiperActiveRobot(
            self.active_config(action_watchdog_s=0.03, feedback_watchdog_s=0.03)
        )
        interface = self.attach_fake_hardware(robot)
        robot.arm_for_motion()
        time.sleep(0.08)
        robot._latch_fault("second failure", request_emergency_stop=True)
        emergency_calls = [call for call in interface.calls if call[0] == "EmergencyStop"]
        self.assertEqual(emergency_calls, [("EmergencyStop", (1,))])
        with self.assertRaisesRegex(ActiveSafetyError, "Fault is latched"):
            robot.send_action(dict(zip(FEATURES, CENTER)))
        robot.disconnect()

    def test_read_only_plugins_have_no_control_calls_and_active_uses_allowlist(self) -> None:
        forbidden = {
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
        for package in ("lerobot_robot_piper", "lerobot_teleoperator_piper"):
            for path in (ROOT / "plugins" / package).rglob("*.py"):
                tree = ast.parse(path.read_text(encoding="utf-8"))
                calls = {
                    node.func.attr
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                }
                self.assertFalse(calls & forbidden, f"{path}: {calls & forbidden}")

        active_path = (
            ROOT
            / "plugins"
            / "lerobot_robot_piper_active"
            / "lerobot_robot_piper_active"
            / "robot_piper_active.py"
        )
        tree = ast.parse(active_path.read_text(encoding="utf-8"))
        interface_calls = {
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Attribute)
            and node.func.value.attr == "_interface"
        }
        self.assertEqual(
            interface_calls,
            {"EnablePiper", "ModeCtrl", "JointCtrl", "GripperCtrl", "EmergencyStop"},
        )
        source = active_path.read_text(encoding="utf-8")
        for forbidden_text in ("ResetPiper(", "MasterSlaveConfig(", "SetInstructionResponse(", ".send("):
            self.assertNotIn(forbidden_text, source)


if __name__ == "__main__":
    unittest.main()
