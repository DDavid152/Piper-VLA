#!/usr/bin/env python3

"""Validate the structural and numeric quality of the first Piper trial."""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import av
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset


DEFAULT_REPO_ID = "local/piper_purple_bag_two_handle_lift_trial_v1"
DEFAULT_ROOT = Path(
    "/home/ubuntu22/Piper-VLA/datasets/"
    "piper_purple_bag_two_handle_lift_trial_v1"
)
EXPECTED_TASK = (
    "夹住紫色手提袋顶部订合在一起的两根橙黄色提带，将袋子竖直提离桌面约"
    "10厘米，保持悬空2秒，再将袋子放回原位，松开提带并将夹爪退离。"
)
EXPECTED_VIDEO_KEYS = (
    "observation.images.front",
    "observation.images.wrist",
)
EXPECTED_VECTOR_KEYS = ("observation.state", "action")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check the first purple-bag Piper trial for expected metadata, "
            "vectors, timing, and decodable camera videos."
        )
    )
    parser.add_argument("--repo-id", default=DEFAULT_REPO_ID)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--expected-task", default=EXPECTED_TASK)
    parser.add_argument("--expected-fps", type=float, default=30.0)
    parser.add_argument("--minimum-effective-fps", type=float, default=28.0)
    parser.add_argument("--minimum-frames", type=int, default=560)
    parser.add_argument("--maximum-frames", type=int, default=620)
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional JSON path for the complete machine-readable result.",
    )
    return parser.parse_args()


def decode_video(path: Path) -> dict[str, Any]:
    frame_count = 0
    width: int | None = None
    height: int | None = None
    with av.open(path) as container:
        stream = container.streams.video[0]
        for frame in container.decode(stream):
            frame_count += 1
            if width is None:
                width = frame.width
                height = frame.height
    return {
        "path": str(path),
        "frames": frame_count,
        "width": width,
        "height": height,
    }


def add_check(
    checks: list[dict[str, Any]],
    name: str,
    passed: bool,
    details: Any,
) -> None:
    checks.append({"name": name, "passed": bool(passed), "details": details})


def main() -> int:
    args = parse_args()
    if args.minimum_frames <= 0 or args.maximum_frames < args.minimum_frames:
        raise ValueError("The accepted frame-count range is invalid.")
    if args.minimum_effective_fps <= 0 or args.expected_fps <= 0:
        raise ValueError("FPS limits must be positive.")
    if not args.root.is_dir():
        print(f"Dataset directory does not exist: {args.root}", file=sys.stderr)
        return 2

    dataset = LeRobotDataset(args.repo_id, root=args.root)
    checks: list[dict[str, Any]] = []
    add_check(checks, "one_episode", dataset.num_episodes == 1, dataset.num_episodes)
    add_check(
        checks,
        "metadata_fps",
        math.isclose(float(dataset.fps), args.expected_fps),
        dataset.fps,
    )
    add_check(
        checks,
        "frame_count",
        args.minimum_frames <= dataset.num_frames <= args.maximum_frames,
        {
            "actual": dataset.num_frames,
            "minimum": args.minimum_frames,
            "maximum": args.maximum_frames,
        },
    )

    features = set(dataset.features)
    expected_features = set(EXPECTED_VIDEO_KEYS + EXPECTED_VECTOR_KEYS)
    add_check(
        checks,
        "required_features",
        expected_features.issubset(features),
        {
            "required": sorted(expected_features),
            "actual": sorted(features),
        },
    )

    tasks = [] if dataset.meta.tasks is None else list(dataset.meta.tasks.index)
    add_check(
        checks,
        "task_text",
        tasks == [args.expected_task],
        {"expected": args.expected_task, "actual": tasks},
    )

    table = dataset.hf_dataset
    columns = set(table.column_names)
    vector_reports: dict[str, Any] = {}
    for key in EXPECTED_VECTOR_KEYS:
        if key not in columns:
            add_check(checks, f"{key}_present", False, sorted(columns))
            continue
        values = np.asarray(table[key], dtype=np.float64)
        finite = bool(np.isfinite(values).all())
        shape_ok = values.ndim == 2 and values.shape == (dataset.num_frames, 7)
        ranges = (
            np.ptp(values, axis=0).astype(float).tolist()
            if values.ndim == 2 and values.shape[1] == 7
            else None
        )
        varied = bool(ranges is not None and max(ranges) >= 0.5)
        vector_reports[key] = {
            "shape": list(values.shape),
            "finite": finite,
            "ranges": ranges,
            "varied": varied,
        }
        add_check(
            checks,
            f"{key}_quality",
            finite and shape_ok and varied,
            vector_reports[key],
        )

    timestamps = np.asarray(table["timestamp"], dtype=np.float64)
    timestamp_deltas = np.diff(timestamps)
    monotonic = bool(
        len(timestamps) == dataset.num_frames
        and np.isfinite(timestamps).all()
        and (timestamp_deltas > 0).all()
    )
    duration_s = (
        float(timestamps[-1] - timestamps[0]) if len(timestamps) >= 2 else 0.0
    )
    effective_fps = (
        float((len(timestamps) - 1) / duration_s) if duration_s > 0 else 0.0
    )
    add_check(
        checks,
        "timestamp_quality",
        monotonic and effective_fps >= args.minimum_effective_fps,
        {
            "monotonic": monotonic,
            "duration_s": duration_s,
            "effective_fps": effective_fps,
            "minimum_effective_fps": args.minimum_effective_fps,
        },
    )

    frame_indices = np.asarray(table["frame_index"], dtype=np.int64)
    episode_indices = np.asarray(table["episode_index"], dtype=np.int64)
    add_check(
        checks,
        "indices",
        bool(
            np.array_equal(frame_indices, np.arange(dataset.num_frames))
            and (episode_indices == 0).all()
        ),
        {
            "first_frame_index": (
                int(frame_indices[0]) if len(frame_indices) else None
            ),
            "last_frame_index": (
                int(frame_indices[-1]) if len(frame_indices) else None
            ),
            "episode_indices": np.unique(episode_indices).astype(int).tolist(),
        },
    )

    video_paths = sorted(args.root.rglob("*.mp4"))
    video_reports: dict[str, Any] = {}
    for key in EXPECTED_VIDEO_KEYS:
        matches = [path for path in video_paths if key in path.as_posix()]
        if len(matches) != 1:
            add_check(
                checks,
                f"{key}_video",
                False,
                {"matching_files": [str(path) for path in matches]},
            )
            continue
        report = decode_video(matches[0])
        video_reports[key] = report
        frame_difference = abs(int(report["frames"]) - dataset.num_frames)
        add_check(
            checks,
            f"{key}_video",
            bool(
                report["width"] == 640
                and report["height"] == 480
                and frame_difference <= 2
            ),
            {**report, "dataset_frame_difference": frame_difference},
        )

    passed = all(check["passed"] for check in checks)
    result = {
        "passed": passed,
        "repo_id": args.repo_id,
        "root": str(args.root),
        "checks": checks,
        "vectors": vector_reports,
        "videos": video_reports,
        "manual_checks_required": [
            "夹爪夹住的是订合在一起的两根橙黄色提带。",
            "袋底竖直提离桌面约10厘米，并连续悬空约2秒。",
            "袋子回到标记区域，夹爪松开提带并退离。",
            "全过程无滑脱、倾倒、明显遮挡或人员进入画面。",
            "录制前后主机CAN TX计数增量为0。",
        ],
    }
    payload = json.dumps(result, ensure_ascii=False, indent=2)
    print(payload)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
