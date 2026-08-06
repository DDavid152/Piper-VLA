#!/usr/bin/env python3

"""Generate a versioned Piper action-safety baseline from a clean dataset."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pandas as pd


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET = (
    PROJECT_ROOT
    / "datasets"
    / "piper_purple_bag_two_handle_lift_manual_v1_clean"
)
DEFAULT_OUTPUT = PROJECT_ROOT / "config" / "piper_safety_baseline_v1.json"
FEATURES = [f"joint_{index}.pos" for index in range(1, 7)] + ["gripper.pos"]
PHYSICAL_MIN = [-150.0, 0.0, -170.0, -100.0, -70.0, -120.0, -5.0]
PHYSICAL_MAX = [150.0, 180.0, 0.0, 100.0, 70.0, 120.0, 120.0]
POSES = {
    "center": {"episode": 48},
    "edge_a": {"episode": 0},
    "edge_b": {"episode": 30},
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def load_data(root: Path) -> pd.DataFrame:
    paths = sorted((root / "data").rglob("*.parquet"))
    if not paths:
        raise FileNotFoundError(f"No Parquet data files under {root / 'data'}")
    return pd.concat((pd.read_parquet(path) for path in paths), ignore_index=True)


def stack(series: pd.Series) -> np.ndarray:
    values = np.stack(series.map(lambda value: np.asarray(value, dtype=np.float64)))
    if values.ndim != 2 or values.shape[1] != len(FEATURES):
        raise ValueError(f"Expected Nx7 vectors, received {values.shape}.")
    if not np.isfinite(values).all():
        raise ValueError("Dataset contains NaN or Inf values.")
    return values


def vector(values: np.ndarray) -> list[float]:
    return [float(value) for value in values]


def file_hashes(root: Path) -> dict[str, str]:
    paths = sorted((root / "data").rglob("*.parquet"))
    paths += sorted((root / "meta").rglob("*.json"))
    paths += sorted((root / "meta").rglob("*.parquet"))
    result: dict[str, str] = {}
    for path in paths:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        result[str(path.relative_to(root))] = digest.hexdigest()
    return result


def build_baseline(root: Path) -> dict[str, object]:
    info = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))
    data = load_data(root)
    action = stack(data["action"])
    state = stack(data["observation.state"])

    deltas = []
    first_states = []
    poses: dict[str, object] = {}
    for episode, episode_data in data.groupby("episode_index", sort=True):
        episode_action = stack(episode_data["action"])
        episode_state = stack(episode_data["observation.state"])
        first_states.append(episode_state[0])
        if len(episode_action) > 1:
            deltas.append(np.abs(np.diff(episode_action, axis=0)))
        for label, pose in POSES.items():
            if int(episode) == pose["episode"]:
                poses[label] = {
                    "episode": int(episode),
                    "state": vector(episode_state[0]),
                }

    if len(poses) != len(POSES):
        raise ValueError(f"Could not resolve all required poses: {sorted(poses)}")
    delta = np.concatenate(deltas, axis=0)
    first = np.stack(first_states)

    return {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "units": ["degree"] * 6 + ["millimeter"],
        "features": FEATURES,
        "source": {
            "dataset_path": str(root.resolve()),
            "codebase_version": info["codebase_version"],
            "episodes": int(info["total_episodes"]),
            "frames": int(info["total_frames"]),
            "file_sha256": file_hashes(root),
        },
        "physical_limits": {
            "min": PHYSICAL_MIN,
            "max": PHYSICAL_MAX,
            "source": "AgileX Piper SDK V2 joint limits; gripper plugin range",
        },
        "clean_action": {
            "min": vector(action.min(axis=0)),
            "max": vector(action.max(axis=0)),
            "q01": vector(np.quantile(action, 0.01, axis=0)),
            "q99": vector(np.quantile(action, 0.99, axis=0)),
        },
        "absolute_action_delta": {
            "p99": vector(np.quantile(delta, 0.99, axis=0)),
            "p99_9": vector(np.quantile(delta, 0.999, axis=0)),
            "max": vector(delta.max(axis=0)),
            "episode_boundaries_excluded": True,
        },
        "initial_state": {
            "min": vector(first.min(axis=0)),
            "max": vector(first.max(axis=0)),
            "tolerance": [1.0] * 6 + [1.0],
        },
        "qa_poses": poses,
    }


def main() -> int:
    args = parse_args()
    baseline = build_baseline(args.dataset)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(baseline, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
