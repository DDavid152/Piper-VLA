import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import can
import draccus
from lerobot.robots import RobotConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.scripts.lerobot_record import RecordConfig
from lerobot.teleoperators import TeleoperatorConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot_robot_piper import PiperRobot, PiperRobotConfig
from lerobot_teleoperator_piper import (
    PiperMasterTeleoperator,
    PiperMasterTeleoperatorConfig,
)


FEATURES = tuple([f"joint_{index}.pos" for index in range(1, 7)] + ["gripper.pos"])


def encode_pair(first: int, second: int) -> bytearray:
    return bytearray(
        first.to_bytes(4, byteorder="big", signed=True)
        + second.to_bytes(4, byteorder="big", signed=True)
    )


def message(arbitration_id: int, data: bytearray) -> can.Message:
    return can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=False,
    )


class FakePiperInterface:
    def __init__(self) -> None:
        now = time.time()
        error_status = SimpleNamespace(
            communication_status_joint_1=False,
            communication_status_joint_2=False,
            communication_status_joint_3=False,
            communication_status_joint_4=False,
            communication_status_joint_5=False,
            communication_status_joint_6=False,
            joint_1_angle_limit=False,
            joint_2_angle_limit=False,
            joint_3_angle_limit=False,
            joint_4_angle_limit=False,
            joint_5_angle_limit=False,
            joint_6_angle_limit=False,
        )
        self.status = SimpleNamespace(
            time_stamp=now,
            Hz=200.0,
            arm_status=SimpleNamespace(
                arm_status=0,
                err_code=0,
                err_status=error_status,
            ),
        )
        self.joints = SimpleNamespace(
            time_stamp=now,
            Hz=200.0,
            joint_state=SimpleNamespace(
                joint_1=1000,
                joint_2=2000,
                joint_3=3000,
                joint_4=4000,
                joint_5=5000,
                joint_6=6000,
            ),
        )
        foc_status = SimpleNamespace(
            voltage_too_low=False,
            motor_overheating=False,
            driver_overcurrent=False,
            driver_overheating=False,
            sensor_status=False,
            driver_error_status=False,
        )
        self.gripper = SimpleNamespace(
            time_stamp=now,
            Hz=200.0,
            gripper_state=SimpleNamespace(
                grippers_angle=25000,
                foc_status=foc_status,
            ),
        )

    def get_connect_status(self) -> bool:
        return True

    def isOk(self) -> bool:
        return True

    def GetArmStatus(self):
        return self.status

    def GetArmJointMsgs(self):
        return self.joints

    def GetArmGripperMsgs(self):
        return self.gripper


class RecoveringPiperInterface(FakePiperInterface):
    def __init__(self) -> None:
        super().__init__()
        stale = time.time() - 1.0
        self.status.time_stamp = stale
        self.joints.time_stamp = stale
        self.gripper.time_stamp = stale
        self.status_reads = 0

    def GetArmStatus(self):
        self.status_reads += 1
        if self.status_reads == 2:
            fresh = time.time()
            self.status.time_stamp = fresh
            self.joints.time_stamp = fresh
            self.gripper.time_stamp = fresh
        return self.status


class PiperPluginTest(unittest.TestCase):
    def test_recording_template_decodes_into_third_party_configs(self) -> None:
        register_third_party_plugins()
        template = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "record_piper_native.example.yaml"
        )
        config = draccus.parse(RecordConfig, config_path=template, args=[])
        self.assertIsInstance(config.robot, PiperRobotConfig)
        self.assertIsInstance(config.teleop, PiperMasterTeleoperatorConfig)
        self.assertEqual(sorted(config.robot.cameras), ["front", "wrist"])
        self.assertEqual(config.robot.control_chain, "native_master_slave")

    def test_purple_bag_trial_config_is_decision_complete(self) -> None:
        register_third_party_plugins()
        trial_config = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "record_piper_purple_bag_lift_trial_v1.yaml"
        )
        config = draccus.parse(RecordConfig, config_path=trial_config, args=[])
        self.assertEqual(
            config.dataset.repo_id,
            "local/piper_purple_bag_two_handle_lift_trial_v1",
        )
        self.assertEqual(config.dataset.fps, 30)
        self.assertEqual(config.dataset.episode_time_s, 20)
        self.assertEqual(config.dataset.num_episodes, 1)
        self.assertFalse(config.dataset.push_to_hub)
        self.assertFalse(config.resume)
        self.assertEqual(sorted(config.robot.cameras), ["front", "wrist"])
        self.assertEqual(config.robot.max_state_age_s, 0.25)
        self.assertEqual(config.robot.state_recovery_timeout_s, 1.0)
        self.assertEqual(config.dataset.rgb_encoder.vcodec, "h264")
        self.assertEqual(config.dataset.rgb_encoder.preset, "ultrafast")
        self.assertEqual(config.dataset.encoder_queue_maxsize, 60)
        self.assertNotIn("REPLACE_WITH", config.dataset.single_task)

    def test_purple_bag_manual_config_is_unlimited_and_operator_delimited(self) -> None:
        register_third_party_plugins()
        manual_config = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "record_piper_purple_bag_lift_manual_v1.yaml"
        )
        config = draccus.parse(RecordConfig, config_path=manual_config, args=[])
        self.assertEqual(
            config.dataset.repo_id,
            "local/piper_purple_bag_two_handle_lift_manual_v1",
        )
        self.assertTrue(config.manual_episode_control)
        self.assertEqual(config.dataset.num_episodes, 0)
        self.assertEqual(config.dataset.fps, 30)
        self.assertFalse(config.dataset.push_to_hub)
        self.assertFalse(config.resume)
        self.assertEqual(sorted(config.robot.cameras), ["front", "wrist"])
        self.assertEqual(config.dataset.rgb_encoder.vcodec, "h264")
        self.assertEqual(config.dataset.rgb_encoder.preset, "ultrafast")
        self.assertEqual(config.dataset.encoder_queue_maxsize, 60)
        self.assertNotIn("REPLACE_WITH", config.dataset.single_task)

    def test_plugins_register_and_factories_instantiate_without_hardware(self) -> None:
        robot_config = PiperRobotConfig()
        teleop_config = PiperMasterTeleoperatorConfig()
        self.assertEqual(robot_config.type, "piper")
        self.assertEqual(teleop_config.type, "piper_master")
        self.assertIs(RobotConfig.get_choice_class("piper"), PiperRobotConfig)
        self.assertIs(
            TeleoperatorConfig.get_choice_class("piper_master"),
            PiperMasterTeleoperatorConfig,
        )
        self.assertIsInstance(make_robot_from_config(robot_config), PiperRobot)
        self.assertIsInstance(
            make_teleoperator_from_config(teleop_config),
            PiperMasterTeleoperator,
        )

    def test_robot_observation_units_and_feature_match(self) -> None:
        robot = PiperRobot(PiperRobotConfig())
        robot._interface = FakePiperInterface()
        observation = robot.get_observation()
        self.assertEqual(tuple(robot.action_features), FEATURES)
        self.assertEqual(tuple(robot.observation_features), FEATURES)
        self.assertEqual(observation["joint_1.pos"], 1.0)
        self.assertEqual(observation["joint_6.pos"], 6.0)
        self.assertEqual(observation["gripper.pos"], 25.0)

    def test_robot_send_action_is_validation_only(self) -> None:
        robot = PiperRobot(PiperRobotConfig())
        robot._interface = FakePiperInterface()
        action = {name: float(index + 1) for index, name in enumerate(FEATURES)}
        self.assertEqual(robot.send_action(action), action)
        self.assertFalse(
            any(
                name.startswith(("JointCtrl", "GripperCtrl", "MotionCtrl"))
                for name in vars(robot._interface)
            )
        )

    def test_robot_rejects_stale_feedback(self) -> None:
        robot = PiperRobot(
            PiperRobotConfig(
                state_recovery_timeout_s=0.03,
                state_retry_interval_s=0.005,
            )
        )
        interface = FakePiperInterface()
        interface.joints.time_stamp = time.time() - 1.0
        robot._interface = interface
        with self.assertRaisesRegex(TimeoutError, "did not recover"):
            robot.get_observation()

    def test_robot_recovers_only_after_fresh_feedback_arrives(self) -> None:
        robot = PiperRobot(
            PiperRobotConfig(
                state_recovery_timeout_s=0.1,
                state_retry_interval_s=0.005,
            )
        )
        interface = RecoveringPiperInterface()
        robot._interface = interface
        observation = robot.get_observation()
        self.assertGreaterEqual(interface.status_reads, 2)
        self.assertEqual(observation["joint_1.pos"], 1.0)

    def test_teleoperator_uses_follower_feedback_before_master_target(self) -> None:
        teleop = PiperMasterTeleoperator(PiperMasterTeleoperatorConfig())
        teleop._process_message(
            message(0x2A1, bytearray.fromhex("0100010000000000"))
        )
        for arbitration_id, values in zip(
            (0x2A5, 0x2A6, 0x2A7),
            ((1000, 2000), (3000, 4000), (5000, 6000)),
        ):
            teleop._process_message(message(arbitration_id, encode_pair(*values)))
        teleop._process_message(message(0x2A8, encode_pair(25000, 0)))
        action = teleop._current_action()
        self.assertEqual(action["joint_1.pos"], 1.0)
        self.assertEqual(action["joint_6.pos"], 6.0)
        self.assertEqual(action["gripper.pos"], 25.0)

    def test_teleoperator_publishes_only_complete_master_triplet(self) -> None:
        teleop = PiperMasterTeleoperator(PiperMasterTeleoperatorConfig())
        for arbitration_id, values in zip(
            (0x155, 0x156),
            ((11000, 12000), (13000, 14000)),
        ):
            teleop._process_message(message(arbitration_id, encode_pair(*values)))
        self.assertIsNone(teleop._joint_target)
        teleop._process_message(message(0x157, encode_pair(15000, 16000)))
        self.assertEqual(teleop._joint_target[0], [11.0, 12.0, 13.0, 14.0, 15.0, 16.0])

    def test_teleoperator_decodes_captured_native_master_frames(self) -> None:
        teleop = PiperMasterTeleoperator(PiperMasterTeleoperatorConfig())
        captured = {
            0x155: "FFFFEDC800000000",
            0x156: "FFFFDE9E00000C3B",
            0x157: "0000247EFFFFEB20",
            0x159: "0000000003E80100",
        }
        for arbitration_id, payload in captured.items():
            teleop._process_message(
                message(arbitration_id, bytearray.fromhex(payload))
            )
        for actual, expected in zip(
            teleop._joint_target[0],
            [-4.664, 0.0, -8.546, 3.131, 9.342, -5.344],
        ):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertEqual(teleop._gripper_target[0], 0.0)

    def test_teleoperator_rejects_unhealthy_follower_status(self) -> None:
        teleop = PiperMasterTeleoperator(PiperMasterTeleoperatorConfig())
        teleop._process_message(
            message(0x2A1, bytearray.fromhex("0101010000000001"))
        )
        with self.assertRaisesRegex(RuntimeError, "arm_status"):
            teleop._current_action()


if __name__ == "__main__":
    unittest.main()
