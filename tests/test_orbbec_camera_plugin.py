import unittest

import numpy as np

from lerobot.cameras import CameraConfig, ColorMode, Cv2Rotation
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot_camera_orbbec import OrbbecCamera, OrbbecCameraConfig


class OrbbecCameraPluginTest(unittest.TestCase):
    def test_config_registers_and_factory_builds_without_connecting(self) -> None:
        config = OrbbecCameraConfig(
            serial_number="TEST_SERIAL",
            width=640,
            height=480,
            fps=30,
        )

        self.assertEqual(config.type, "orbbec")
        self.assertIs(CameraConfig.get_choice_class("orbbec"), OrbbecCameraConfig)
        cameras = make_cameras_from_configs({"front": config})
        self.assertIsInstance(cameras["front"], OrbbecCamera)
        self.assertFalse(cameras["front"].is_connected)

    def test_config_rejects_empty_serial(self) -> None:
        with self.assertRaisesRegex(ValueError, "serial_number"):
            OrbbecCameraConfig(serial_number="", width=640, height=480, fps=30)

    def test_config_rejects_invalid_timeout(self) -> None:
        with self.assertRaisesRegex(ValueError, "timeout_ms"):
            OrbbecCameraConfig(
                serial_number="TEST",
                width=640,
                height=480,
                fps=30,
                timeout_ms=0,
            )

    def test_config_rejects_invalid_warmup(self) -> None:
        with self.assertRaisesRegex(ValueError, "warmup_s"):
            OrbbecCameraConfig(
                serial_number="TEST",
                width=640,
                height=480,
                fps=30,
                warmup_s=-1,
            )

    def test_rotation_keeps_expected_output_shape(self) -> None:
        config = OrbbecCameraConfig(
            serial_number="TEST",
            width=640,
            height=480,
            fps=30,
            color_mode=ColorMode.RGB,
            rotation=Cv2Rotation.ROTATE_90,
        )
        camera = OrbbecCamera(config)
        source = np.zeros((640, 480, 3), dtype=np.uint8)
        self.assertEqual(camera._rotate(source).shape, (480, 640, 3))


if __name__ == "__main__":
    unittest.main()
