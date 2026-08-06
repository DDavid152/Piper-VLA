import json
import logging
import math
import time
from datetime import datetime, timezone
from functools import cached_property
from pathlib import Path
from typing import Any, TextIO

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
        self._validated_action_count = 0
        self._last_observation_state: dict[str, float] | None = None
        self._last_observation_monotonic: float | None = None
        self._last_validated_action: dict[str, float] | None = None
        self._passive_action_log: TextIO | None = None
        self._passive_envelope_warning_count = 0

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

    def _open_passive_action_log(self) -> None:
        if not self.config.passive_action_log_path or self._passive_action_log is not None:
            return
        path = Path(self.config.passive_action_log_path).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._passive_action_log = path.open("x", encoding="utf-8", buffering=1)
        logger.info("Passive action audit log: %s", path)

    def _close_passive_action_log(self) -> None:
        if self._passive_action_log is not None:
            self._passive_action_log.flush()
            self._passive_action_log.close()
            self._passive_action_log = None

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        self._validate_adapter_identity()
        connected_cameras = []
        try:
            self._open_passive_action_log()
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
            self._close_passive_action_log()
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
        self._last_observation_state = {
            feature: float(observation[feature]) for feature in PIPER_FEATURES
        }
        self._last_observation_monotonic = time.monotonic()
        for name, camera in self.cameras.items():
            observation[name] = camera.read_latest()
            if getattr(camera, "use_depth", False):
                observation[f"{name}_depth"] = camera.read_latest_depth()
        return observation

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        """Validate the recorded target without transmitting it to the follower."""
        validated = {key: float(value) for key, value in action.items()}
        envelope_warning = None
        try:
            self._validate_values(validated, source="action")
        except (RuntimeError, ValueError) as exc:
            if not self.config.passive_action_log_path:
                raise
            if set(validated) != set(PIPER_FEATURES) or not all(
                math.isfinite(value) for value in validated.values()
            ):
                raise
            envelope_warning = str(exc)
            self._passive_envelope_warning_count += 1
            if (
                self._passive_envelope_warning_count == 1
                or self._passive_envelope_warning_count % 30 == 0
            ):
                logger.warning(
                    "Passive audit recorded broad-envelope violation %d without sending it: %s",
                    self._passive_envelope_warning_count,
                    exc,
                )
        now_monotonic = time.monotonic()
        if self._last_observation_state is None:
            self._last_observation_state = self._read_arm_state_with_recovery()
            self._last_observation_monotonic = time.monotonic()
            now_monotonic = time.monotonic()
        delta = None
        if self._last_validated_action is not None:
            delta = [
                validated[feature] - self._last_validated_action[feature]
                for feature in PIPER_FEATURES
            ]
        self._open_passive_action_log()
        if self._passive_action_log is not None:
            record = {
                "schema_version": 1,
                "sequence": self._validated_action_count,
                "wall_time_utc": datetime.now(timezone.utc).isoformat(),
                "monotonic_time_s": now_monotonic,
                "state": [self._last_observation_state[key] for key in PIPER_FEATURES],
                "raw_action": [validated[key] for key in PIPER_FEATURES],
                "action_delta": delta,
                "observation_age_s": (
                    now_monotonic - self._last_observation_monotonic
                    if self._last_observation_monotonic is not None
                    else None
                ),
                "plugin_envelope_warning": envelope_warning,
            }
            self._passive_action_log.write(
                json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n"
            )
            if (
                self._validated_action_count + 1
            ) % self.config.passive_action_log_flush_every == 0:
                self._passive_action_log.flush()
        self._last_validated_action = validated.copy()
        self._validated_action_count += 1
        if self._validated_action_count == 1:
            logger.info(
                "Recorded first passive policy action (no CAN transmission): %s",
                validated,
            )
        return validated

    @check_if_not_connected
    def disconnect(self) -> None:
        for camera in reversed(list(self.cameras.values())):
            if camera.is_connected:
                camera.disconnect()
        if self._interface is not None and self._interface.get_connect_status():
            self._interface.DisconnectPort()
        self._interface = None
        self._close_passive_action_log()
        if self.name == "piper":
            logger.info(
                "%s disconnected after validating %d passive action(s) without issuing a robot command.",
                self,
                self._validated_action_count,
            )
        else:
            # Active adapters reuse this class only for feedback, cameras, and
            # resource cleanup. Their subclass logger owns the authoritative
            # action/stop summary; calling those actions "passive" here is false.
            logger.info("%s feedback and camera resources disconnected.", self)
