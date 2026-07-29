import logging
import math
import time
from pathlib import Path
from threading import Event, Lock, Thread
from typing import Any

import can
from lerobot.teleoperators import Teleoperator
from lerobot.types import RobotAction
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .configuration_piper_master import PiperMasterTeleoperatorConfig


logger = logging.getLogger(__name__)

PIPER_FEATURES = tuple([f"joint_{index}.pos" for index in range(1, 7)] + ["gripper.pos"])
JOINT_TARGET_IDS = (0x155, 0x156, 0x157)
JOINT_FEEDBACK_IDS = (0x2A5, 0x2A6, 0x2A7)
CAN_FILTER_IDS = (0x151, *JOINT_TARGET_IDS, 0x159, 0x2A1, *JOINT_FEEDBACK_IDS, 0x2A8)


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


def decode_signed_32(data: bytes | bytearray, start: int) -> int:
    return int.from_bytes(data[start : start + 4], byteorder="big", signed=True)


class PiperMasterTeleoperator(Teleoperator):
    """Passively reconstruct master targets from the shared Piper CAN bus."""

    config_class = PiperMasterTeleoperatorConfig
    name = "piper_master"

    def __init__(self, config: PiperMasterTeleoperatorConfig):
        super().__init__(config)
        self.config = config
        self._bus: can.BusABC | None = None
        self._connected = False
        self._stop_event = Event()
        self._receiver_thread: Thread | None = None
        self._lock = Lock()

        self._joint_feedback_pairs: dict[int, tuple[float, float, float]] = {}
        self._gripper_feedback: tuple[float, float] | None = None
        self._arm_status: tuple[int, int, float] | None = None
        self._pending_target_pairs: dict[int, tuple[float, float, float]] = {}
        self._joint_target: tuple[list[float], float] | None = None
        self._gripper_target: tuple[float, float] | None = None
        self._stats = {
            "frames_received": 0,
            "error_frames": 0,
            "ignored_frames": 0,
            "complete_joint_targets": 0,
        }

    @property
    def action_features(self) -> dict[str, type]:
        return dict.fromkeys(PIPER_FEATURES, float)

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return bool(
            self._connected
            and self._bus is not None
            and self._receiver_thread is not None
            and self._receiver_thread.is_alive()
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        logger.info("Piper master targets use protocol units; no software calibration is run.")

    def configure(self) -> None:
        logger.info("Piper master teleoperator is receive-only and sends no configuration.")

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
        filters = [
            {"can_id": arbitration_id, "can_mask": 0x7FF, "extended": False}
            for arbitration_id in CAN_FILTER_IDS
        ]
        try:
            self._bus = can.Bus(
                interface="socketcan",
                channel=self.config.can_interface,
                receive_own_messages=False,
                can_filters=filters,
            )
            self._stop_event.clear()
            self._connected = True
            self._receiver_thread = Thread(
                target=self._receive_loop,
                name="piper_master_receive",
                daemon=True,
            )
            self._receiver_thread.start()
            self._wait_for_feedback()
            self.configure()
        except Exception:
            self._disconnect_internal()
            raise

    def _wait_for_feedback(self) -> None:
        deadline = time.perf_counter() + self.config.connect_timeout_s
        while time.perf_counter() < deadline:
            try:
                self._current_action()
                return
            except (RuntimeError, TimeoutError, ValueError):
                time.sleep(0.05)
        raise ConnectionError(
            f"No complete, healthy Piper feedback arrived within "
            f"{self.config.connect_timeout_s:.1f}s."
        )

    def _receive_loop(self) -> None:
        if self._bus is None:
            return
        while not self._stop_event.is_set():
            try:
                message = self._bus.recv(timeout=0.1)
            except (can.CanError, OSError) as exc:
                logger.error("Piper master receive loop stopped: %s", exc)
                self._connected = False
                return
            if message is not None:
                self._process_message(message)

    def _process_message(self, message: can.Message) -> None:
        received_at = time.perf_counter()
        with self._lock:
            self._stats["frames_received"] += 1
            if message.is_error_frame:
                self._stats["error_frames"] += 1
                return
            if message.is_extended_id or len(message.data) != 8:
                self._stats["ignored_frames"] += 1
                return

            arbitration_id = message.arbitration_id
            data = message.data
            if arbitration_id == 0x2A1:
                arm_status = int(data[1])
                error_code = int.from_bytes(data[6:8], byteorder="big", signed=False)
                self._arm_status = (arm_status, error_code, received_at)
            elif arbitration_id in JOINT_FEEDBACK_IDS:
                self._joint_feedback_pairs[arbitration_id] = (
                    decode_signed_32(data, 0) * 0.001,
                    decode_signed_32(data, 4) * 0.001,
                    received_at,
                )
            elif arbitration_id == 0x2A8:
                self._gripper_feedback = (
                    decode_signed_32(data, 0) * 0.001,
                    received_at,
                )
            elif arbitration_id in JOINT_TARGET_IDS:
                self._pending_target_pairs[arbitration_id] = (
                    decode_signed_32(data, 0) * 0.001,
                    decode_signed_32(data, 4) * 0.001,
                    received_at,
                )
                self._publish_coherent_joint_target()
            elif arbitration_id == 0x159:
                self._gripper_target = (
                    decode_signed_32(data, 0) * 0.001,
                    received_at,
                )
            elif arbitration_id != 0x151:
                self._stats["ignored_frames"] += 1

    def _publish_coherent_joint_target(self) -> None:
        if not all(arbitration_id in self._pending_target_pairs for arbitration_id in JOINT_TARGET_IDS):
            return
        timestamps = [
            self._pending_target_pairs[arbitration_id][2]
            for arbitration_id in JOINT_TARGET_IDS
        ]
        if (max(timestamps) - min(timestamps)) * 1000 > self.config.coherence_window_ms:
            return
        values: list[float] = []
        for arbitration_id in JOINT_TARGET_IDS:
            first, second, _ = self._pending_target_pairs[arbitration_id]
            values.extend((first, second))
        self._joint_target = (values, max(timestamps))
        self._stats["complete_joint_targets"] += 1

    def _fresh_feedback_values(self, now: float) -> tuple[list[float], float]:
        missing = [
            hex(arbitration_id)
            for arbitration_id in JOINT_FEEDBACK_IDS
            if arbitration_id not in self._joint_feedback_pairs
        ]
        if missing or self._gripper_feedback is None:
            raise TimeoutError(f"Incomplete follower feedback; missing {missing}.")

        values: list[float] = []
        timestamps: list[float] = []
        for arbitration_id in JOINT_FEEDBACK_IDS:
            first, second, timestamp = self._joint_feedback_pairs[arbitration_id]
            values.extend((first, second))
            timestamps.append(timestamp)
        gripper, gripper_timestamp = self._gripper_feedback
        timestamps.append(gripper_timestamp)
        age_s = now - min(timestamps)
        if age_s > self.config.max_feedback_age_s:
            raise TimeoutError(f"Follower feedback is {age_s:.3f}s old.")
        return values, gripper

    def _current_action(self) -> dict[str, float]:
        now = time.perf_counter()
        with self._lock:
            if self._arm_status is None:
                raise TimeoutError("No follower status has been received.")
            arm_status, error_code, status_timestamp = self._arm_status
            status_age_s = now - status_timestamp
            if status_age_s > self.config.max_feedback_age_s:
                raise TimeoutError(f"Follower status is {status_age_s:.3f}s old.")
            if arm_status != 0 or error_code != 0:
                raise RuntimeError(
                    f"Follower reports arm_status={arm_status}, err_code={error_code}."
                )

            feedback_joints, feedback_gripper = self._fresh_feedback_values(now)
            joints = self._joint_target[0] if self._joint_target is not None else feedback_joints
            gripper = (
                self._gripper_target[0]
                if self._gripper_target is not None
                else feedback_gripper
            )
            action = {
                **{
                    f"joint_{index}.pos": float(joints[index - 1])
                    for index in range(1, 7)
                },
                "gripper.pos": float(gripper),
            }
            self._validate_action(action)
            return action

    def _validate_action(self, action: dict[str, float]) -> None:
        if not all(math.isfinite(value) for value in action.values()):
            raise ValueError("Piper master action contains a non-finite value.")
        if self.config.reject_all_zero_action and all(
            abs(value) < 1e-9 for value in action.values()
        ):
            raise RuntimeError("Piper master action is unexpectedly all zero.")
        for index in range(1, 7):
            value = action[f"joint_{index}.pos"]
            if abs(value) > self.config.max_abs_joint_degrees:
                raise ValueError(f"joint_{index}.pos={value} exceeds the safety envelope.")
        gripper = action["gripper.pos"]
        if not self.config.gripper_min_mm <= gripper <= self.config.gripper_max_mm:
            raise ValueError(f"gripper.pos={gripper} is outside the safety envelope.")

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        return self._current_action()

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, Any]) -> None:
        if feedback:
            logger.debug("Ignoring feedback in passive Piper master mode.")

    def get_health_stats(self) -> dict[str, Any]:
        with self._lock:
            return {
                **self._stats,
                "connected": self.is_connected,
                "action_source": (
                    "master_target" if self._joint_target is not None else "follower_feedback"
                ),
                "joint_target_received": self._joint_target is not None,
                "gripper_target_received": self._gripper_target is not None,
            }

    def _disconnect_internal(self) -> None:
        self._connected = False
        self._stop_event.set()
        if self._receiver_thread is not None and self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=1.0)
        if self._bus is not None:
            self._bus.shutdown()
        self._receiver_thread = None
        self._bus = None

    def disconnect(self) -> None:
        if self._bus is None and self._receiver_thread is None:
            return
        self._disconnect_internal()
        logger.info("Piper master teleoperator disconnected without transmitting a CAN frame.")
