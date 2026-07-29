import importlib.util
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
VIEWER_PATH = PROJECT_ROOT / "scripts" / "view_piper_dataset_dashboard.py"
SPEC = importlib.util.spec_from_file_location(
    "view_piper_dataset_dashboard",
    VIEWER_PATH,
)
assert SPEC is not None and SPEC.loader is not None
dashboard = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(dashboard)


class FakeRobot:
    def __init__(self, config) -> None:
        self.config = config
        self.is_connected = False

    def connect(self, calibrate=False) -> None:
        del calibrate
        self.is_connected = True

    def get_observation(self) -> dict[str, float]:
        return {
            **{
                f"joint_{index}.pos": float(index)
                for index in range(1, 7)
            },
            "gripper.pos": 20.0,
        }

    def disconnect(self) -> None:
        self.is_connected = False


class FakeTeleoperator:
    def __init__(self, config) -> None:
        self.config = config
        self.is_connected = False
        self.samples = 0

    def connect(self, calibrate=False) -> None:
        del calibrate
        self.is_connected = True

    def get_action(self) -> dict[str, float]:
        self.samples += 1
        return {
            **{
                f"joint_{index}.pos": float(index) + 0.5
                for index in range(1, 7)
            },
            "gripper.pos": 22.0,
        }

    def get_health_stats(self) -> dict:
        return {
            "connected": self.is_connected,
            "frames_received": self.samples * 8,
            "action_source": "master_target",
        }

    def disconnect(self) -> None:
        self.is_connected = False


class DatasetDashboardTest(unittest.TestCase):
    def test_passive_source_builds_complete_training_vectors(self) -> None:
        source = dashboard.PiperDataSource(
            PROJECT_ROOT / "config" / "piper_native_master_slave.json",
            refresh_fps=100.0,
        )
        source._interface_preflight = lambda: None
        with (
            patch.object(dashboard, "PiperRobot", FakeRobot),
            patch.object(dashboard, "PiperMasterTeleoperator", FakeTeleoperator),
        ):
            source.start()
            time.sleep(0.05)
            status = source.status()
            source.stop()

        self.assertTrue(status["connected"])
        self.assertGreater(status["sample_count"], 0)
        self.assertEqual(tuple(status["observation"]), dashboard.PIPER_FEATURES)
        self.assertEqual(tuple(status["action"]), dashboard.PIPER_FEATURES)
        self.assertEqual(status["delta"]["joint_1.pos"], 0.5)
        self.assertEqual(status["delta"]["gripper.pos"], 2.0)
        self.assertEqual(status["health"]["action_source"], "master_target")
        self.assertTrue(status["can"]["receive_only"])

    def test_page_lists_all_training_sample_fields(self) -> None:
        state = SimpleNamespace(
            cameras={
                "front": SimpleNamespace(serial_number="FRONT"),
                "wrist": SimpleNamespace(serial_number="WRIST"),
            },
            raw_config={
                "cameras": {
                    "front": {"physical_role": "remote_fixed"},
                    "wrist": {"physical_role": "end_effector"},
                }
            },
            task="将物块放入目标区域",
        )
        page = dashboard.build_index_html(state).decode("utf-8")

        self.assertIn("observation.images.front", page)
        self.assertIn("observation.images.wrist", page)
        self.assertIn("Follower observation", page)
        self.assertIn("Master action", page)
        self.assertIn("joint_6.pos", page)
        self.assertIn("gripper.pos", page)
        self.assertIn("timestamp", page)
        self.assertIn("episode_index", page)
        self.assertIn("将物块放入目标区域", page)


if __name__ == "__main__":
    unittest.main()
