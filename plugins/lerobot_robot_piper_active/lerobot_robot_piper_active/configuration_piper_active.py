from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig


PROJECT_ROOT = Path("/home/ubuntu22/Piper-VLA")
DEFAULT_BASELINE_PATH = PROJECT_ROOT / "config" / "piper_safety_baseline_v1.json"
DEFAULT_CALIBRATION_PATH = PROJECT_ROOT / "config" / "piper_active_calibration_v1.json"
MOTION_CONFIRMATION = "I_UNDERSTAND_PIPER_WILL_MOVE"
CALIBRATION_GENERATOR = "lerobot_robot_piper_active.calibration"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@RobotConfig.register_subclass("piper_active")
@dataclass
class PiperActiveRobotConfig(RobotConfig):
    """Fail-closed direct follower control configuration."""

    can_interface: str = "can0"
    expected_adapter_serial: str = "002900225547571120343930"
    control_chain: str = "direct_sdk_follower"
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    motion_enabled: bool = False
    arm_on_first_action: bool = False
    operator_confirmation: str = ""
    start_pose_mode: str = "training_envelope"
    safety_profile: str = "strict"
    max_active_actions: int = 0
    max_motion_duration_s: float = 0.0
    max_joint_displacement_deg: float = 5.0
    max_gripper_displacement_mm: float = 15.0
    enforce_displacement_window: bool = True
    safety_baseline_path: str = str(DEFAULT_BASELINE_PATH)
    calibration_path: str = str(DEFAULT_CALIBRATION_PATH)
    motion_speed_percent: int = 10
    gripper_effort: int = 500
    feedback_watchdog_s: float = 0.25
    action_watchdog_s: float = 0.25
    external_control_quiet_s: float = 2.0
    enable_timeout_s: float = 5.0
    arm_hold_s: float = 0.5
    arm_hold_command_hz: float = 20.0
    arm_hold_tolerance_deg: float = 0.25
    arm_acquisition_max_drift_deg: float = 1.0
    arm_acquisition_stability_deg: float = 0.05
    connect_timeout_s: float = 3.0
    max_state_age_s: float = 0.25
    state_recovery_timeout_s: float = 1.0
    state_retry_interval_s: float = 0.01
    min_feedback_hz: float = 20.0
    reject_all_zero_state: bool = True
    max_abs_joint_degrees: float = 360.0
    gripper_min_mm: float = -5.0
    gripper_max_mm: float = 120.0
    passive_action_log_path: str = ""
    passive_action_log_flush_every: int = 30
    active_command_log_path: str = ""
    active_command_log_flush_every: int = 1

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.control_chain != "direct_sdk_follower":
            raise ValueError("piper_active only supports direct_sdk_follower.")
        if not self.can_interface.strip() or not self.expected_adapter_serial.strip():
            raise ValueError("CAN interface and expected adapter serial are required.")
        if not 1 <= self.motion_speed_percent <= 100:
            raise ValueError("`motion_speed_percent` must be in [1, 100].")
        if self.safety_profile not in {"strict", "micro_observe"}:
            raise ValueError("`safety_profile` must be `strict` or `micro_observe`.")
        if self.start_pose_mode not in {"training_envelope", "current_physical"}:
            raise ValueError(
                "`start_pose_mode` must be `training_envelope` or `current_physical`."
            )
        if (
            isinstance(self.max_active_actions, bool)
            or not isinstance(self.max_active_actions, int)
            or self.max_active_actions < 0
        ):
            raise ValueError("`max_active_actions` must be a non-negative integer.")
        if not math.isfinite(self.max_motion_duration_s) or self.max_motion_duration_s < 0:
            raise ValueError("`max_motion_duration_s` must be non-negative.")
        if (
            not math.isfinite(self.max_joint_displacement_deg)
            or self.max_joint_displacement_deg <= 0
        ):
            raise ValueError("`max_joint_displacement_deg` must be positive.")
        if (
            not math.isfinite(self.max_gripper_displacement_mm)
            or self.max_gripper_displacement_mm <= 0
        ):
            raise ValueError("`max_gripper_displacement_mm` must be positive.")
        if not isinstance(self.enforce_displacement_window, bool):
            raise ValueError("`enforce_displacement_window` must be a boolean.")
        if not 0 <= self.gripper_effort <= 5000:
            raise ValueError("`gripper_effort` must be in [0, 5000].")
        if self.feedback_watchdog_s <= 0 or self.action_watchdog_s <= 0:
            raise ValueError("Feedback and action watchdogs must be positive.")
        if self.external_control_quiet_s < 2.0:
            raise ValueError("External-control silence must be checked for at least 2 seconds.")
        if not math.isfinite(self.enable_timeout_s) or self.enable_timeout_s <= 0:
            raise ValueError("`enable_timeout_s` must be positive.")
        if not math.isfinite(self.arm_hold_s) or self.arm_hold_s < 0.05:
            raise ValueError("`arm_hold_s` must be at least 0.05 seconds.")
        if (
            not math.isfinite(self.arm_hold_command_hz)
            or not 5 <= self.arm_hold_command_hz <= 50
        ):
            raise ValueError("`arm_hold_command_hz` must be in [5, 50].")
        if (
            not math.isfinite(self.arm_hold_tolerance_deg)
            or not 0 < self.arm_hold_tolerance_deg <= 0.5
        ):
            raise ValueError("`arm_hold_tolerance_deg` must be in (0, 0.5].")
        if (
            not math.isfinite(self.arm_acquisition_max_drift_deg)
            or not self.arm_hold_tolerance_deg
            < self.arm_acquisition_max_drift_deg
            <= 2.0
        ):
            raise ValueError(
                "`arm_acquisition_max_drift_deg` must be greater than the "
                "strict hold tolerance and at most 2 degrees."
            )
        if (
            not math.isfinite(self.arm_acquisition_stability_deg)
            or not 0 < self.arm_acquisition_stability_deg <= 0.1
        ):
            raise ValueError(
                "`arm_acquisition_stability_deg` must be in (0, 0.1]."
            )
        if self.max_state_age_s <= 0 or self.min_feedback_hz <= 0:
            raise ValueError("Feedback freshness and rate limits must be positive.")
        if self.passive_action_log_flush_every <= 0:
            raise ValueError("`passive_action_log_flush_every` must be positive.")
        if self.active_command_log_flush_every <= 0:
            raise ValueError("`active_command_log_flush_every` must be positive.")

        baseline_path = Path(self.safety_baseline_path).expanduser()
        if not baseline_path.is_file():
            raise ValueError(f"Safety baseline is missing: {baseline_path}")
        baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
        if baseline.get("schema_version") != 1:
            raise ValueError("Unsupported Piper safety-baseline schema.")

        if self.motion_enabled:
            if self.operator_confirmation != MOTION_CONFIRMATION:
                raise ValueError(
                    "Active motion requires the exact operator confirmation string."
                )
            calibration_path = Path(self.calibration_path).expanduser()
            if not calibration_path.is_file():
                raise ValueError(f"Active calibration is missing: {calibration_path}")
            calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
            if (
                calibration.get("schema_version") != 2
                or calibration.get("calibration_version") != 2
            ):
                raise ValueError(
                    "Active motion requires a generated Piper v2 calibration."
                )
            if calibration.get("verified") is not True:
                raise ValueError(
                    "Active calibration is not verified; refusing before any CAN transmit path."
                )
            calibration_serial = calibration.get("adapter", {}).get("serial")
            if calibration_serial != self.expected_adapter_serial:
                raise ValueError("Calibration adapter identity does not match this config.")
            verification = calibration.get("verification", {})
            if verification.get("generator") != CALIBRATION_GENERATOR:
                raise ValueError("Piper v2 calibration lacks generator provenance.")
            evidence = verification.get("evidence", {})
            for name in ("passive_mapping", "commissioning"):
                record = evidence.get(name, {})
                digest = record.get("sha256")
                evidence_path_value = record.get("path")
                if not isinstance(digest, str) or len(digest) != 64:
                    raise ValueError(f"Piper v2 calibration lacks {name} evidence.")
                if not isinstance(evidence_path_value, str):
                    raise ValueError(f"Piper v2 calibration lacks the {name} evidence path.")
                evidence_path = Path(evidence_path_value).expanduser()
                if not evidence_path.is_file():
                    raise ValueError(f"Piper v2 {name} evidence is missing: {evidence_path}")
                if _sha256(evidence_path) != digest:
                    raise ValueError(f"Piper v2 {name} evidence hash does not match.")
