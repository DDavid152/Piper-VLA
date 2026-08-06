from __future__ import annotations

import argparse
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


FEATURES = tuple([f"joint_{index}.pos" for index in range(1, 7)] + ["gripper.pos"])
GENERATOR = "lerobot_robot_piper_active.calibration"
MIN_PASSIVE_SAMPLES = 20
JOINT_PROTOCOL_SCALE_TOLERANCE = 0.02
JOINT_PROTOCOL_OFFSET_MAX_DEG = 1.0
JOINT_TRACKING_MAX_ERROR_DEG = 2.0
GRIPPER_TRACKING_MAX_ERROR_MM = 1.0
COMMISSIONING_DELTA_DEG = 0.5
COMMISSIONING_MAX_ERROR_DEG = 0.25
PIPER_JOINT_MIN_DEG = (-150.0, 0.0, -170.0, -100.0, -70.0, -120.0)
PIPER_JOINT_MAX_DEG = (150.0, 180.0, 0.0, 100.0, 70.0, 120.0)
MODEL_GRIPPER_MIN_MM = -5.0
MODEL_GRIPPER_MAX_MM = 120.0
PIPER_GRIPPER_MIN_MM = 0.0
PIPER_GRIPPER_MAX_MM = 70.0
REQUIRED_MASTER_GRIPPER_LOW_MM = 0.5
REQUIRED_MASTER_GRIPPER_HIGH_MM = 100.0
REQUIRED_MASTER_GRIPPER_LINEAR_MIN_MM = 20.0
REQUIRED_MASTER_GRIPPER_LINEAR_MAX_MM = 50.0


class CalibrationEvidenceError(ValueError):
    """Raw calibration evidence is incomplete, inconsistent, or unsafe."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise CalibrationEvidenceError(
                    f"{path}:{line_number} is not valid JSON."
                ) from exc
            if not isinstance(record, dict):
                raise CalibrationEvidenceError(
                    f"{path}:{line_number} must contain a JSON object."
                )
            records.append(record)
    if not records:
        raise CalibrationEvidenceError(f"Calibration evidence is empty: {path}")
    return records


def _finite_vector(value: Any, *, label: str, length: int) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise CalibrationEvidenceError(f"{label} must contain {length} values.")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise CalibrationEvidenceError(f"{label} contains NaN or Inf.")
    return result


def _fit_linear(inputs: list[float], outputs: list[float], *, label: str) -> dict[str, float]:
    mean_input = sum(inputs) / len(inputs)
    mean_output = sum(outputs) / len(outputs)
    denominator = sum((value - mean_input) ** 2 for value in inputs)
    if denominator <= 1e-12:
        raise CalibrationEvidenceError(f"{label} does not span enough input positions.")
    scale = sum(
        (input_value - mean_input) * (output_value - mean_output)
        for input_value, output_value in zip(inputs, outputs, strict=True)
    ) / denominator
    offset = mean_output - scale * mean_input
    if not math.isfinite(scale) or not math.isfinite(offset) or scale == 0:
        raise CalibrationEvidenceError(f"{label} produced an invalid mapping.")
    max_error = max(
        abs(scale * input_value + offset - output_value)
        for input_value, output_value in zip(inputs, outputs, strict=True)
    )
    return {"scale": scale, "offset": offset, "max_error": max_error}


def _validate_passive_mapping(
    path: Path,
    *,
    expected_adapter_serial: str,
) -> tuple[list[dict[str, Any]], list[dict[str, float]], dict[str, float]]:
    records = _load_jsonl(path)
    if len(records) < MIN_PASSIVE_SAMPLES:
        raise CalibrationEvidenceError(
            f"Passive mapping requires at least {MIN_PASSIVE_SAMPLES} samples."
        )
    master_samples: list[list[float]] = []
    follower_samples: list[list[float]] = []
    for index, record in enumerate(records):
        if record.get("schema_version") != 1:
            raise CalibrationEvidenceError(f"Passive record {index} has the wrong schema.")
        if record.get("record_type") != "piper_passive_mapping":
            raise CalibrationEvidenceError(f"Passive record {index} has the wrong type.")
        if record.get("capture_mode") != "read_only":
            raise CalibrationEvidenceError(
                f"Passive record {index} was not captured in read-only mode."
            )
        if record.get("adapter_serial") != expected_adapter_serial:
            raise CalibrationEvidenceError(
                f"Passive record {index} has the wrong USB-CAN identity."
            )
        master = _finite_vector(
            record.get("master"), label=f"passive master {index}", length=7
        )
        follower = _finite_vector(
            record.get("follower"), label=f"passive follower {index}", length=7
        )
        for joint_index in range(6):
            for label, vector in (("master", master), ("follower", follower)):
                if not (
                    PIPER_JOINT_MIN_DEG[joint_index]
                    <= vector[joint_index]
                    <= PIPER_JOINT_MAX_DEG[joint_index]
                ):
                    raise CalibrationEvidenceError(
                        f"Passive {label} joint_{joint_index + 1} sample {index} "
                        "is outside a physical limit."
                    )
        if not MODEL_GRIPPER_MIN_MM <= master[6] <= MODEL_GRIPPER_MAX_MM:
            raise CalibrationEvidenceError(
                f"Passive master gripper sample {index} is outside a physical limit."
            )
        if not PIPER_GRIPPER_MIN_MM <= follower[6] <= PIPER_GRIPPER_MAX_MM:
            raise CalibrationEvidenceError(
                f"Passive follower gripper sample {index} is outside an SDK physical limit."
            )
        master_samples.append(master)
        follower_samples.append(follower)

    joint_mappings: list[dict[str, float]] = []
    for joint_index in range(6):
        inputs = [sample[joint_index] for sample in master_samples]
        outputs = [sample[joint_index] for sample in follower_samples]
        if max(inputs) - min(inputs) < 1.0:
            raise CalibrationEvidenceError(
                f"joint_{joint_index + 1} passive span is below 1 degree."
            )
        fit = _fit_linear(inputs, outputs, label=f"joint_{joint_index + 1}")
        if abs(fit["scale"] - 1.0) > JOINT_PROTOCOL_SCALE_TOLERANCE:
            raise CalibrationEvidenceError(
                f"joint_{joint_index + 1} target/feedback scale {fit['scale']:.6f} "
                "does not match the Piper protocol's identity coordinates."
            )
        if abs(fit["offset"]) > JOINT_PROTOCOL_OFFSET_MAX_DEG:
            raise CalibrationEvidenceError(
                f"joint_{joint_index + 1} target/feedback offset "
                f"{fit['offset']:+.3f} degrees exceeds "
                f"{JOINT_PROTOCOL_OFFSET_MAX_DEG:.3f}."
            )
        tracking_max_error = max(
            abs(input_value - output_value)
            for input_value, output_value in zip(inputs, outputs, strict=True)
        )
        if tracking_max_error > JOINT_TRACKING_MAX_ERROR_DEG:
            raise CalibrationEvidenceError(
                f"joint_{joint_index + 1} settled target/feedback error "
                f"{tracking_max_error:.3f} degrees exceeds "
                f"{JOINT_TRACKING_MAX_ERROR_DEG:.3f}."
            )
        # 0x155/0x156/0x157 and JointCtrl use the same 0.001-degree target
        # coordinates. Follower feedback is tracking evidence, not a second
        # command coordinate system to regress into the target.
        joint_mappings.append(
            {
                "scale": 1.0,
                "offset": 0.0,
                "max_error": tracking_max_error,
                "measured_scale": fit["scale"],
                "measured_offset": fit["offset"],
            }
        )

    gripper_inputs = [sample[6] for sample in master_samples]
    gripper_outputs = [sample[6] for sample in follower_samples]
    if (
        min(gripper_inputs) > REQUIRED_MASTER_GRIPPER_LOW_MM
        or max(gripper_inputs) < REQUIRED_MASTER_GRIPPER_HIGH_MM
    ):
        raise CalibrationEvidenceError(
            "Gripper passive mapping must cover approximately closed through fully open."
        )
    if not any(
        REQUIRED_MASTER_GRIPPER_LINEAR_MIN_MM
        <= value
        <= REQUIRED_MASTER_GRIPPER_LINEAR_MAX_MM
        for value in gripper_inputs
    ):
        raise CalibrationEvidenceError(
            "Gripper passive mapping needs a settled sample in the 20-50 mm "
            "linear region."
        )
    expected_gripper_outputs = [
        min(max(value, PIPER_GRIPPER_MIN_MM), PIPER_GRIPPER_MAX_MM)
        for value in gripper_inputs
    ]
    gripper_tracking_max_error = max(
        abs(expected - actual)
        for expected, actual in zip(
            expected_gripper_outputs, gripper_outputs, strict=True
        )
    )
    if gripper_tracking_max_error > GRIPPER_TRACKING_MAX_ERROR_MM:
        raise CalibrationEvidenceError(
            f"Gripper settled target/feedback error "
            f"{gripper_tracking_max_error:.3f} mm exceeds "
            f"{GRIPPER_TRACKING_MAX_ERROR_MM:.3f}."
        )
    gripper_mapping = {
        "scale": 1.0,
        "offset": 0.0,
        "max_error": gripper_tracking_max_error,
    }
    return records, joint_mappings, gripper_mapping


def _validate_commissioning(
    path: Path,
    *,
    expected_adapter_serial: str,
) -> list[dict[str, Any]]:
    records = _load_jsonl(path)
    directions: set[tuple[int, int]] = set()
    for index, record in enumerate(records):
        if record.get("schema_version") != 1:
            raise CalibrationEvidenceError(
                f"Commissioning record {index} has the wrong schema."
            )
        if record.get("record_type") != "piper_commissioning":
            raise CalibrationEvidenceError(
                f"Commissioning record {index} has the wrong type."
            )
        if record.get("adapter_serial") != expected_adapter_serial:
            raise CalibrationEvidenceError(
                f"Commissioning record {index} has the wrong USB-CAN identity."
            )
        joint = record.get("joint")
        if isinstance(joint, bool) or not isinstance(joint, int) or not 1 <= joint <= 6:
            raise CalibrationEvidenceError(
                f"Commissioning record {index} has an invalid joint number."
            )
        requested = float(record.get("requested_delta_degrees", math.nan))
        measured = float(record.get("measured_delta_degrees", math.nan))
        other_drift = float(record.get("other_joint_max_abs_delta_degrees", math.nan))
        if not all(math.isfinite(value) for value in (requested, measured, other_drift)):
            raise CalibrationEvidenceError(
                f"Commissioning record {index} contains NaN or Inf."
            )
        if not math.isclose(abs(requested), COMMISSIONING_DELTA_DEG, abs_tol=1e-9):
            raise CalibrationEvidenceError(
                f"Commissioning record {index} is not a fixed +/-0.5 degree test."
            )
        direction = 1 if requested > 0 else -1
        if measured * direction <= 0 or abs(measured - requested) > COMMISSIONING_MAX_ERROR_DEG:
            raise CalibrationEvidenceError(
                f"Commissioning record {index} failed direction/tracking tolerance."
            )
        if other_drift > COMMISSIONING_MAX_ERROR_DEG:
            raise CalibrationEvidenceError(
                f"Commissioning record {index} moved another joint too far."
            )
        speed = record.get("motion_speed_percent")
        if isinstance(speed, bool) or not isinstance(speed, int) or not 1 <= speed <= 10:
            raise CalibrationEvidenceError(
                f"Commissioning record {index} did not use 1-10% motion speed."
            )
        if record.get("can_error_count") != 0:
            raise CalibrationEvidenceError(
                f"Commissioning record {index} reports a CAN error."
            )
        if record.get("emergency_stop_verified") is not True:
            raise CalibrationEvidenceError(
                f"Commissioning record {index} lacks emergency-stop verification."
            )
        key = (joint, direction)
        if key in directions:
            raise CalibrationEvidenceError(
                f"Commissioning contains a duplicate joint/direction test: {key}."
            )
        directions.add(key)

    expected = {(joint, direction) for joint in range(1, 7) for direction in (-1, 1)}
    if directions != expected:
        missing = sorted(expected - directions)
        raise CalibrationEvidenceError(
            f"Commissioning must pass both +/-0.5 degree tests for all joints; missing {missing}."
        )
    return records


def build_verified_calibration(
    passive_mapping_path: str | Path,
    commissioning_path: str | Path,
    *,
    expected_adapter_serial: str,
    can_interface: str,
    operator: str,
) -> dict[str, Any]:
    """Derive a verified v2 document only from passing raw evidence."""
    if not operator.strip():
        raise CalibrationEvidenceError("A non-empty commissioning operator is required.")
    passive_path = Path(passive_mapping_path).expanduser().resolve()
    commissioning_evidence_path = Path(commissioning_path).expanduser().resolve()
    if not passive_path.is_file() or not commissioning_evidence_path.is_file():
        raise CalibrationEvidenceError("Both calibration evidence files must exist.")

    passive_records, joint_fits, gripper_fit = _validate_passive_mapping(
        passive_path,
        expected_adapter_serial=expected_adapter_serial,
    )
    commissioning_records = _validate_commissioning(
        commissioning_evidence_path,
        expected_adapter_serial=expected_adapter_serial,
    )

    # 0x159 and GripperCtrl both use 0.001 mm. The installed official SDK's
    # enabled gripper limit clamps the small Piper gripper to [0, 70] mm.
    # Preserve the model's native target units and represent that clamp as a
    # monotonic piecewise mapping rather than an incorrect global rescaling.
    gripper_points = [
        {"input_mm": MODEL_GRIPPER_MIN_MM, "output_mm": PIPER_GRIPPER_MIN_MM},
        {"input_mm": PIPER_GRIPPER_MAX_MM, "output_mm": PIPER_GRIPPER_MAX_MM},
        {"input_mm": MODEL_GRIPPER_MAX_MM, "output_mm": PIPER_GRIPPER_MAX_MM},
    ]

    return {
        "schema_version": 2,
        "calibration_version": 2,
        "verified": True,
        "units": {
            "joint_input": "degree",
            "joint_sdk": "0.001_degree",
            "gripper_input": "millimeter",
            "gripper_sdk": "0.001_millimeter",
        },
        "adapter": {
            "interface": can_interface,
            "serial": expected_adapter_serial,
        },
        "sdk_physical_limits": {
            "joint_min_degrees": list(PIPER_JOINT_MIN_DEG),
            "joint_max_degrees": list(PIPER_JOINT_MAX_DEG),
            "gripper_min_mm": PIPER_GRIPPER_MIN_MM,
            "gripper_max_mm": PIPER_GRIPPER_MAX_MM,
        },
        "joints": [
            {
                "name": FEATURES[index],
                "scale": round(fit["scale"], 12),
                "offset_degrees": round(fit["offset"], 12),
            }
            for index, fit in enumerate(joint_fits)
        ],
        "gripper_points": gripper_points,
        "verification": {
            "generator": GENERATOR,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "operator": operator.strip(),
            "passive_sample_count": len(passive_records),
            "commissioning_test_count": len(commissioning_records),
            "thresholds": {
                "joint_protocol_scale_tolerance": JOINT_PROTOCOL_SCALE_TOLERANCE,
                "joint_protocol_offset_max_degrees": JOINT_PROTOCOL_OFFSET_MAX_DEG,
                "joint_tracking_max_error_degrees": JOINT_TRACKING_MAX_ERROR_DEG,
                "gripper_tracking_max_error_mm": GRIPPER_TRACKING_MAX_ERROR_MM,
                "commissioning_delta_degrees": COMMISSIONING_DELTA_DEG,
                "commissioning_max_error_degrees": COMMISSIONING_MAX_ERROR_DEG,
            },
            "mapping_basis": {
                "joints": "Piper 0x155/0x156/0x157 and JointCtrl identity units",
                "gripper": "Piper 0x159/GripperCtrl identity units clamped to 0-70 mm",
            },
            "passive_tracking": {
                "joint_max_errors_degrees": [
                    round(fit["max_error"], 9) for fit in joint_fits
                ],
                "gripper_max_error_mm": round(gripper_fit["max_error"], 9),
            },
            "evidence": {
                "passive_mapping": {
                    "path": str(passive_path),
                    "sha256": _sha256(passive_path),
                },
                "commissioning": {
                    "path": str(commissioning_evidence_path),
                    "sha256": _sha256(commissioning_evidence_path),
                },
            },
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a verified Piper active-calibration v2 file from read-only "
            "master/follower mapping and fixed +/-0.5 degree commissioning evidence."
        )
    )
    parser.add_argument("--passive-mapping", required=True, type=Path)
    parser.add_argument("--commissioning", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--expected-adapter-serial", required=True)
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument("--operator", required=True)
    args = parser.parse_args(argv)

    document = build_verified_calibration(
        args.passive_mapping,
        args.commissioning,
        expected_adapter_serial=args.expected_adapter_serial,
        can_interface=args.can_interface,
        operator=args.operator,
    )
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(f"Generated verified Piper v2 calibration: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
