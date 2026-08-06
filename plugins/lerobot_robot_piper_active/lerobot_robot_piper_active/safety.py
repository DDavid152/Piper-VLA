from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np


FEATURES = tuple([f"joint_{index}.pos" for index in range(1, 7)] + ["gripper.pos"])


class ActiveSafetyError(RuntimeError):
    """An active command failed a fail-closed safety condition."""


class PiperSafetyProcessor:
    """Validate, slew-limit, calibrate, and convert a seven-axis action."""

    def __init__(
        self,
        baseline_path: str | Path,
        calibration_path: str | Path,
        *,
        profile: str = "strict",
        start_pose_mode: str = "training_envelope",
        max_joint_displacement_deg: float = 5.0,
        max_gripper_displacement_mm: float = 15.0,
        enforce_displacement_window: bool = True,
    ):
        self.baseline_path = Path(baseline_path).expanduser().resolve()
        self.calibration_path = Path(calibration_path).expanduser().resolve()
        self.baseline = json.loads(self.baseline_path.read_text(encoding="utf-8"))
        self.calibration = json.loads(self.calibration_path.read_text(encoding="utf-8"))
        if profile not in {"strict", "micro_observe"}:
            raise ActiveSafetyError(f"Unsupported safety profile: {profile!r}.")
        if start_pose_mode not in {"training_envelope", "current_physical"}:
            raise ActiveSafetyError(f"Unsupported start-pose mode: {start_pose_mode!r}.")
        self.profile = profile
        self.start_pose_mode = start_pose_mode
        self.enforce_displacement_window = enforce_displacement_window
        self.max_displacement = np.asarray(
            [max_joint_displacement_deg] * 6 + [max_gripper_displacement_mm],
            dtype=np.float64,
        )
        if not np.isfinite(self.max_displacement).all() or bool(
            (self.max_displacement <= 0).any()
        ):
            raise ActiveSafetyError("Micro-observe displacement limits must be positive.")
        self._validate_documents()
        self.last_safe_action: np.ndarray | None = None
        self.initial_state: np.ndarray | None = None

    def _validate_documents(self) -> None:
        if self.baseline.get("schema_version") != 1:
            raise ActiveSafetyError("Unsupported safety-baseline schema.")
        if tuple(self.baseline.get("features", ())) != FEATURES:
            raise ActiveSafetyError("Safety-baseline features do not match Piper.")
        if self.calibration.get("schema_version") not in {1, 2}:
            raise ActiveSafetyError("Unsupported calibration schema.")
        joints = self.calibration.get("joints", [])
        if len(joints) != 6:
            raise ActiveSafetyError("Calibration must contain six joint mappings.")
        for index, mapping in enumerate(joints, 1):
            scale = mapping.get("scale")
            offset = mapping.get("offset_degrees")
            if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in (scale, offset)):
                raise ActiveSafetyError(f"Joint {index} calibration is invalid.")
            if scale == 0:
                raise ActiveSafetyError(f"Joint {index} calibration scale cannot be zero.")
        points = self.calibration.get("gripper_points", [])
        if len(points) < 3:
            raise ActiveSafetyError("Calibration requires at least three gripper mapping points.")
        inputs = [float(point["input_mm"]) for point in points]
        outputs = [float(point["output_mm"]) for point in points]
        if not all(math.isfinite(value) for value in (*inputs, *outputs)):
            raise ActiveSafetyError("Gripper calibration contains non-finite values.")
        if not all(right > left for left, right in zip(inputs, inputs[1:])):
            raise ActiveSafetyError("Gripper calibration inputs must be strictly increasing.")
        output_deltas = np.diff(np.asarray(outputs))
        if self.calibration.get("schema_version") == 2:
            if bool((output_deltas < 0).any()) or not bool((output_deltas > 0).any()):
                raise ActiveSafetyError(
                    "Piper v2 gripper calibration outputs must be nondecreasing "
                    "with at least one increasing segment."
                )
        elif not (
            bool((output_deltas > 0).all()) or bool((output_deltas < 0).all())
        ):
            raise ActiveSafetyError("Gripper calibration outputs must be strictly monotonic.")
        limits = self.calibration.get("sdk_physical_limits")
        if self.calibration.get("schema_version") == 2:
            if not isinstance(limits, dict):
                raise ActiveSafetyError("Piper v2 calibration lacks SDK physical limits.")
            joint_min = np.asarray(limits.get("joint_min_degrees", []), dtype=np.float64)
            joint_max = np.asarray(limits.get("joint_max_degrees", []), dtype=np.float64)
            if (
                joint_min.shape != (6,)
                or joint_max.shape != (6,)
                or not np.isfinite(joint_min).all()
                or not np.isfinite(joint_max).all()
                or bool((joint_min >= joint_max).any())
            ):
                raise ActiveSafetyError("Piper v2 joint SDK physical limits are invalid.")
            gripper_limits = (
                limits.get("gripper_min_mm"),
                limits.get("gripper_max_mm"),
            )
            if not all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in gripper_limits
            ) or gripper_limits[0] >= gripper_limits[1]:
                raise ActiveSafetyError("Piper v2 gripper SDK physical limits are invalid.")

    @property
    def calibration_verified(self) -> bool:
        return (
            self.calibration.get("schema_version") == 2
            and self.calibration.get("calibration_version") == 2
            and self.calibration.get("verified") is True
        )

    def validate_initial_state(self, state: dict[str, float]) -> None:
        values = self._vector(state, "initial state")
        physical_min = np.asarray(self.baseline["physical_limits"]["min"])
        physical_max = np.asarray(self.baseline["physical_limits"]["max"])
        if bool(((values < physical_min) | (values > physical_max)).any()):
            raise ActiveSafetyError("Initial state is outside a physical limit.")
        if self.start_pose_mode == "training_envelope":
            initial_min = np.asarray(self.baseline["initial_state"]["min"])
            initial_max = np.asarray(self.baseline["initial_state"]["max"])
            tolerance = np.asarray(self.baseline["initial_state"]["tolerance"])
            if bool(
                ((values < initial_min - tolerance) | (values > initial_max + tolerance)).any()
            ):
                raise ActiveSafetyError(
                    "Initial state is outside the 51-episode start envelope."
                )
        self.last_safe_action = values.copy()
        self.initial_state = values.copy()

    def _vector(self, values: dict[str, float], label: str) -> np.ndarray:
        if set(values) != set(FEATURES):
            raise ActiveSafetyError(f"{label} must contain exactly the seven Piper features.")
        vector = np.asarray([float(values[feature]) for feature in FEATURES], dtype=np.float64)
        if not np.isfinite(vector).all():
            raise ActiveSafetyError(f"{label} contains NaN or Inf.")
        return vector

    def validate_raw_action(self, action: dict[str, float]) -> np.ndarray:
        """Validate only the invariant schema, finiteness, and physical limits."""
        raw = self._vector(action, "action")
        physical_min = np.asarray(self.baseline["physical_limits"]["min"])
        physical_max = np.asarray(self.baseline["physical_limits"]["max"])
        if bool(((raw < physical_min) | (raw > physical_max)).any()):
            raise ActiveSafetyError("Action is outside a physical limit.")
        return raw

    def preview(
        self,
        action: dict[str, float],
        *,
        initial_state: dict[str, float] | None = None,
    ) -> dict[str, Any]:
        """Evaluate an action without advancing the limiter state."""
        previous_action = (
            None if self.last_safe_action is None else self.last_safe_action.copy()
        )
        previous_initial = None if self.initial_state is None else self.initial_state.copy()
        try:
            if initial_state is not None:
                self.validate_initial_state(initial_state)
            return self.prepare(action)
        finally:
            self.last_safe_action = previous_action
            self.initial_state = previous_initial

    def prepare(self, action: dict[str, float]) -> dict[str, Any]:
        raw = self._vector(action, "action")
        physical_min = np.asarray(self.baseline["physical_limits"]["min"])
        physical_max = np.asarray(self.baseline["physical_limits"]["max"])
        physical_violation = (raw < physical_min) | (raw > physical_max)
        warnings: list[str] = []
        if bool(physical_violation.any()):
            message = "Raw action is outside a physical limit."
            if self.profile == "strict":
                raise ActiveSafetyError("Action is outside a physical limit.")
            warnings.append(message + " It was clipped before slew limiting.")
        physical_clipped = np.clip(raw, physical_min, physical_max)
        task_min = np.asarray(self.baseline["clean_action"]["min"])
        task_max = np.asarray(self.baseline["clean_action"]["max"])
        if bool(((raw < task_min) | (raw > task_max)).any()):
            message = "Action is outside the clean-data task envelope."
            if self.profile == "strict":
                raise ActiveSafetyError(message)
            warnings.append(message)

        limited = physical_clipped.copy()
        was_slew_limited = [False] * 7
        if self.last_safe_action is not None:
            delta = physical_clipped - self.last_safe_action
            hard_max = np.asarray(self.baseline["absolute_action_delta"]["max"])
            if bool((np.abs(delta) > hard_max).any()):
                message = "Action jump exceeds the historical clean-data maximum."
                if self.profile == "strict":
                    raise ActiveSafetyError(message)
                warnings.append(message)
            p99 = np.asarray(self.baseline["absolute_action_delta"]["p99"])
            clipped_delta = np.clip(delta, -p99, p99)
            was_slew_limited = (clipped_delta != delta).tolist()
            limited = self.last_safe_action + clipped_delta

        was_displacement_clipped = [False] * 7
        if self.profile == "micro_observe" and self.enforce_displacement_window:
            if self.initial_state is None:
                raise ActiveSafetyError(
                    "micro_observe requires a validated initial state before any action."
                )
            window_min = np.maximum(
                self.initial_state - self.max_displacement,
                physical_min,
            )
            window_max = np.minimum(
                self.initial_state + self.max_displacement,
                physical_max,
            )
            displacement_clipped = np.clip(limited, window_min, window_max)
            displacement_violation = displacement_clipped != limited
            was_displacement_clipped = displacement_violation.tolist()
            if bool(displacement_violation.any()):
                warnings.append(
                    "Slew-limited action reached the micro-observe displacement "
                    "window and was clipped."
                )
                limited = displacement_clipped

        calibrated_joint_degrees = []
        for index, mapping in enumerate(self.calibration["joints"]):
            calibrated_joint_degrees.append(
                limited[index] * float(mapping["scale"])
                + float(mapping["offset_degrees"])
            )

        if self.calibration.get("schema_version") == 2:
            limits = self.calibration["sdk_physical_limits"]
            joint_min = np.asarray(limits["joint_min_degrees"], dtype=np.float64)
            joint_max = np.asarray(limits["joint_max_degrees"], dtype=np.float64)
            calibrated_vector = np.asarray(calibrated_joint_degrees, dtype=np.float64)
            if bool(((calibrated_vector < joint_min) | (calibrated_vector > joint_max)).any()):
                raise ActiveSafetyError("Calibrated joint action is outside an SDK physical limit.")
        joint_units = [
            int(round(calibrated_degrees * 1000.0))
            for calibrated_degrees in calibrated_joint_degrees
        ]

        points = self.calibration["gripper_points"]
        input_points = np.asarray([point["input_mm"] for point in points], dtype=np.float64)
        output_points = np.asarray([point["output_mm"] for point in points], dtype=np.float64)
        gripper_mm = float(np.interp(limited[6], input_points, output_points))
        if self.calibration.get("schema_version") == 2:
            limits = self.calibration["sdk_physical_limits"]
            if not limits["gripper_min_mm"] <= gripper_mm <= limits["gripper_max_mm"]:
                raise ActiveSafetyError("Calibrated gripper action is outside an SDK physical limit.")
        gripper_units = int(round(gripper_mm * 1000.0))
        self.last_safe_action = limited.copy()
        return {
            "raw": raw.tolist(),
            "physical_clipped": physical_clipped.tolist(),
            "was_physical_clipped": physical_violation.tolist(),
            "limited": limited.tolist(),
            "was_slew_limited": was_slew_limited,
            "was_displacement_clipped": was_displacement_clipped,
            "warnings": warnings,
            "calibrated_joint_degrees": calibrated_joint_degrees,
            "calibrated_gripper_mm": gripper_mm,
            "joint_units_0_001_degree": joint_units,
            "gripper_units_0_001_mm": gripper_units,
        }
