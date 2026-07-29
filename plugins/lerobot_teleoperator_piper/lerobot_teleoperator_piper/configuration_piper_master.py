from dataclasses import dataclass
from lerobot.teleoperators import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("piper_master")
@dataclass
class PiperMasterTeleoperatorConfig(TeleoperatorConfig):
    """Passive master target reader for Piper native master/slave mode."""

    can_interface: str = "can0"
    expected_adapter_serial: str = "002900225547571120343930"
    # See the robot config: this remains a string for draccus compatibility
    # and is strictly checked below.
    control_chain: str = "native_master_slave"
    connect_timeout_s: float = 3.0
    max_feedback_age_s: float = 0.25
    coherence_window_ms: float = 50.0
    reject_all_zero_action: bool = True
    max_abs_joint_degrees: float = 360.0
    gripper_min_mm: float = -5.0
    gripper_max_mm: float = 120.0

    def __post_init__(self) -> None:
        if not self.can_interface.strip():
            raise ValueError("`can_interface` must not be empty.")
        if not self.expected_adapter_serial.strip():
            raise ValueError("`expected_adapter_serial` must not be empty.")
        if self.control_chain != "native_master_slave":
            raise ValueError("The Piper teleoperator only supports native_master_slave.")
        if self.connect_timeout_s <= 0:
            raise ValueError("`connect_timeout_s` must be positive.")
        if self.max_feedback_age_s <= 0:
            raise ValueError("`max_feedback_age_s` must be positive.")
        if self.coherence_window_ms <= 0:
            raise ValueError("`coherence_window_ms` must be positive.")
        if self.max_abs_joint_degrees <= 0:
            raise ValueError("`max_abs_joint_degrees` must be positive.")
        if self.gripper_min_mm >= self.gripper_max_mm:
            raise ValueError("The gripper range is invalid.")
