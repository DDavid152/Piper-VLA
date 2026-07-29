from dataclasses import dataclass, field
from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("piper")
@dataclass
class PiperRobotConfig(RobotConfig):
    """Read-only follower configuration for Piper native master/slave recording."""

    can_interface: str = "can0"
    expected_adapter_serial: str = "002900225547571120343930"
    # Keep this as `str`: draccus 0.11 cannot decode Literal fields nested in
    # third-party choice configs. __post_init__ still enforces the only value
    # this passive plugin supports.
    control_chain: str = "native_master_slave"
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    connect_timeout_s: float = 3.0
    max_state_age_s: float = 0.25
    min_feedback_hz: float = 20.0
    reject_all_zero_state: bool = True
    max_abs_joint_degrees: float = 360.0
    gripper_min_mm: float = -5.0
    gripper_max_mm: float = 120.0

    def __post_init__(self) -> None:
        super().__post_init__()
        if not self.can_interface.strip():
            raise ValueError("`can_interface` must not be empty.")
        if not self.expected_adapter_serial.strip():
            raise ValueError("`expected_adapter_serial` must not be empty.")
        if self.control_chain != "native_master_slave":
            raise ValueError("The Piper robot plugin only supports native_master_slave.")
        if self.connect_timeout_s <= 0:
            raise ValueError("`connect_timeout_s` must be positive.")
        if self.max_state_age_s <= 0:
            raise ValueError("`max_state_age_s` must be positive.")
        if self.min_feedback_hz <= 0:
            raise ValueError("`min_feedback_hz` must be positive.")
        if self.max_abs_joint_degrees <= 0:
            raise ValueError("`max_abs_joint_degrees` must be positive.")
        if self.gripper_min_mm >= self.gripper_max_mm:
            raise ValueError("The gripper range is invalid.")
