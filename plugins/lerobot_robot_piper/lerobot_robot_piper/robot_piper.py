import logging
import math
import time
from functools import cached_property
from pathlib import Path
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from piper_sdk import C_PiperInterface_V2

from .configuration_piper import PiperRobotConfig


logger = logging.getLogger(__name__)

PIPER_FEATURES = tuple([f"joint_{index}.pos" for index in range(1, 7)] + ["gripper.pos"])
GRIPPER_ERROR_FIELDS = (
    "voltage_too_low",
    "motor_overheating",
    "driver_overcurrent",
    "driver_overheating",
    "sensor_status",
    "driver_error_status",
)


def read_usb_serial_for_network_interface(interface: str) -> str:
    device_path = Path("/sys/class/net") / interface / "device"
    if not device_path.exists():
        raise ConnectionError(f"CAN interface {interface!r} does not exist.")

    current = device_path.resolve()
    for parent in (current, *current.parents):
        serial_path = parent / "serial"
        if serial_path.is_file():
            return serial_path.read_text(encoding="utf-8").strip()
    raise ConnectionError(f"Could not resolve a USB serial number for {interface!r}.")


class PiperRobot(Robot):
    """Read follower feedback and cameras without transmitting Piper commands."""

    config_class = PiperRobotConfig
    name = "piper"

    def __init__(self, config: PiperRobotConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self._interface: Any | None = None

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = dict.fromkeys(PIPER_FEATURES, float)
        for name, camera in self.cameras.items():
            features[name] = (camera.height, camera.width, 3)
            if getattr(camera, "use_depth", False):
                features[f"{name}_depth"] = (camera.height, camera.width, 1)
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        return dict.fromkeys(PIPER_FEATURES, float)

    @property
    def is_connected(self) -> bool:
        sdk_connected = bool(
            self._interface is not None and self._interface.get_connect_status()
        )
        return sdk_connected and all(camera.is_connected for camera in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        logger.info("Piper native master/slave mode does not perform software calibration.")

    def configure(self) -> None:
        logger.info("Piper read-only plugin does not configure, enable, or home the arm.")

    def _validate_adapter_identity(self) -> None:
        actual = read_usb_serial_for_network_interface(self.config.can_interface)
        if actual != self.config.expected_adapter_serial:
            raise ConnectionError(
                f"{self.config.can_interface!r} is backed by USB-CAN serial {actual!r}; "
                f"expected shared-bus adapter {self.config.expected_adapter_serial!r}."
            )

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        self._validate_adapter_identity()
        connected_cameras = []
        try:
            interface = C_PiperInterface_V2(
                self.config.can_interface,
                judge_flag=True,
                can_auto_init=True,
                start_sdk_joint_limit=True,
                start_sdk_gripper_limit=True,
            )
            interface.ConnectPort(piper_init=False, start_thread=True)
            self._interface = interface
            self._wait_for_valid_feedback()

            for camera in self.cameras.values():
                camera.connect()
                connected_cameras.append(camera)
            self.configure()
            logger.info("%s connected in passive native master/slave mode.", self)
        except Exception:
            for camera in reversed(connected_cameras):
                if camera.is_connected:
                    camera.disconnect()
            if self._interface is not None and self._interface.get_connect_status():
                self._interface.DisconnectPort()
            self._interface = None
            raise

    def _wait_for_valid_feedback(self) -> None:
        deadline = time.perf_counter() + self.config.connect_timeout_s
        last_error: Exception | None = None
        while time.perf_counter() < deadline:
            try:
                self._read_arm_state()
                return
            except (RuntimeError, TimeoutError, ValueError) as exc:
                last_error = exc
                time.sleep(0.05)
        raise ConnectionError(
            f"No valid Piper follower feedback arrived within "
            f"{self.config.connect_timeout_s:.1f}s."
        ) from last_error

    def _validate_wrapper(self, name: str, wrapper: Any) -> None:
        timestamp = float(wrapper.time_stamp)
        if timestamp <= 0:
            raise TimeoutError(f"{name} has no timestamp.")
        age_s = time.time() - timestamp
        if age_s < -0.1 or age_s > self.config.max_state_age_s:
            raise TimeoutError(
                f"{name} is stale or invalid: timestamp age is {age_s:.3f}s."
            )
        hz = float(wrapper.Hz)
        if hz < self.config.min_feedback_hz:
            raise RuntimeError(
                f"{name} frequency {hz:.1f}Hz is below "
                f"{self.config.min_feedback_hz:.1f}Hz."
            )

    def _read_arm_state(self) -> dict[str, float]:
        if self._interface is None:
            raise RuntimeError("Piper SDK interface has not been created.")
        if not self._interface.isOk():
            raise RuntimeError("Piper SDK reports an unhealthy CAN receive thread.")

        status = self._interface.GetArmStatus()
        joints = self._interface.GetArmJointMsgs()
        gripper = self._interface.GetArmGripperMsgs()
        self._validate_wrapper("arm status", status)
        self._validate_wrapper("joint feedback", joints)
        self._validate_wrapper("gripper feedback", gripper)

        arm_status = status.arm_status
        if int(arm_status.arm_status) != 0 or int(arm_status.err_code) != 0:
            raise RuntimeError(
                f"Piper follower reports arm_status={arm_status.arm_status}, "
                f"err_code={arm_status.err_code}."
            )

        foc_status = gripper.gripper_state.foc_status
        gripper_errors = [
            field for field in GRIPPER_ERROR_FIELDS if bool(getattr(foc_status, field))
        ]
        if gripper_errors:
            raise RuntimeError(f"Piper gripper reports errors: {gripper_errors}.")

        joint_state = joints.joint_state
        values = {
            f"joint_{index}.pos": float(getattr(joint_state, f"joint_{index}")) * 0.001
            for index in range(1, 7)
        }
        values["gripper.pos"] = float(gripper.gripper_state.grippers_angle) * 0.001
        self._validate_values(values, source="observation")
        return values

    def _read_arm_state_with_recovery(self) -> dict[str, float]:
        """Return only fresh feedback, retrying a transient host-side stall."""
        deadline = time.perf_counter() + self.config.state_recovery_timeout_s
        first_error: TimeoutError | None = None
        while True:
            try:
                state = self._read_arm_state()
                if first_error is not None:
                    logger.info(
                        "Piper feedback recovered after a transient host stall."
                    )
                return state
            except TimeoutError as exc:
                if first_error is None:
                    first_error = exc
                    logger.warning(
                        "Piper feedback is temporarily stale; waiting up to "
                        "%.2fs for a fresh sample.",
                        self.config.state_recovery_timeout_s,
                    )
                if time.perf_counter() >= deadline:
                    raise TimeoutError(
                        "Piper feedback did not recover within "
                        f"{self.config.state_recovery_timeout_s:.2f}s."
                    ) from exc
                time.sleep(self.config.state_retry_interval_s)

    def _validate_values(self, values: dict[str, float], *, source: str) -> None:
        if set(values) != set(PIPER_FEATURES):
            raise ValueError(
                f"Piper {source} keys must be exactly {list(PIPER_FEATURES)}; "
                f"received {sorted(values)}."
            )
        if not all(math.isfinite(float(value)) for value in values.values()):
            raise ValueError(f"Piper {source} contains a non-finite value.")
        if self.config.reject_all_zero_state and all(
            abs(float(value)) < 1e-9 for value in values.values()
        ):
            raise RuntimeError(f"Piper {source} is unexpectedly all zero.")
        for index in range(1, 7):
            value = float(values[f"joint_{index}.pos"])
            if abs(value) > self.config.max_abs_joint_degrees:
                raise ValueError(f"joint_{index}.pos={value} exceeds the safety envelope.")
        gripper = float(values["gripper.pos"])
        if not self.config.gripper_min_mm <= gripper <= self.config.gripper_max_mm:
            raise ValueError(f"gripper.pos={gripper} is outside the safety envelope.")

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        observation: RobotObservation = self._read_arm_state_with_recovery()
        for name, camera in self.cameras.items():
            observation[name] = camera.read_latest()
            if getattr(camera, "use_depth", False):
                observation[f"{name}_depth"] = camera.read_latest_depth()
        return observation

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        """Validate the recorded target without transmitting it to the follower."""
        validated = {key: float(value) for key, value in action.items()}
        self._validate_values(validated, source="action")
        return validated

    @check_if_not_connected
    def disconnect(self) -> None:
        for camera in reversed(list(self.cameras.values())):
            if camera.is_connected:
                camera.disconnect()
        if self._interface is not None and self._interface.get_connect_status():
            self._interface.DisconnectPort()
        self._interface = None
        logger.info("%s disconnected without issuing a robot command.", self)
