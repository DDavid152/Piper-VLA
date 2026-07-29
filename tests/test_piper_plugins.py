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
        robot = PiperRobot(PiperRobotConfig())
        interface = FakePiperInterface()
        interface.joints.time_stamp = time.time() - 1.0
        robot._interface = interface
        with self.assertRaisesRegex(TimeoutError, "joint feedback"):
            robot.get_observation()

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
