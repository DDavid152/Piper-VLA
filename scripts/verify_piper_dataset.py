#!/usr/bin/env python3

"""Batch QA for any number of Piper LeRobot episodes.

The checker understands LeRobot's layout where multiple episodes may share one
physical MP4. It validates each episode against its Parquet row range and video
timestamp interval, then produces both per-episode and dataset-wide results.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

import av
import numpy as np
import pandas as pd


DEFAULT_REPO_ID = "local/piper_purple_bag_two_handle_lift_manual_v1"
DEFAULT_ROOT = Path(
    "/home/ubuntu22/Piper-VLA/datasets/"
    "piper_purple_bag_two_handle_lift_manual_v1"
)
DEFAULT_TASK = (
    "夹住紫色手提袋顶部订合在一起的两根橙黄色提带，将袋子竖直提离桌面约"
    "10厘米，保持悬空2秒，再将袋子放回原位，松开提带并将夹爪退离。"
)
VIDEO_KEYS = (
    "observation.images.front",
    "observation.images.wrist",
)
VECTOR_KEYS = ("observation.state", "action")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate every episode in a Piper LeRobot dataset, including shared "
            "MP4 frame ranges, timing, 7-D vectors, tasks, and index continuity."
        )
    )
    parser.add_argument("--repo-id", default=DEFAULT_REPO_ID)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--expected-task", default=DEFAULT_TASK)
    parser.add_argument("--expected-fps", type=float, default=30.0)
    parser.add_argument("--minimum-effective-fps", type=float, default=28.0)
    parser.add_argument("--expected-width", type=int, default=640)
    parser.add_argument("--expected-height", type=int, default=480)
    parser.add_argument("--minimum-vector-range", type=float, default=0.5)
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional path for the complete JSON report.",
    )
    parser.add_argument(
        "--print-json",
        action="store_true",
        help="Print the complete JSON report instead of the concise summary.",
    )
    return parser.parse_args()


def add_check(
    checks: list[dict[str, Any]],
    name: str,
    passed: bool,
    details: Any,
) -> None:
    checks.append({"name": name, "passed": bool(passed), "details": details})


def read_parquet_tree(directory: Path) -> pd.DataFrame:
    paths = sorted(directory.rglob("*.parquet"))
    if not paths:
        raise FileNotFoundError(f"No Parquet files found under {directory}")
    return pd.concat((pd.read_parquet(path) for path in paths), ignore_index=True)


def normalize_tasks(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, np.ndarray):
        value = value.tolist()
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value]
    return [str(value)]


def stack_vector(series: pd.Series) -> np.ndarray:
    return np.stack([np.asarray(value, dtype=np.float64) for value in series])


def decode_video(path: Path) -> dict[str, Any]:
    frame_count = 0
    width: int | None = None
    height: int | None = None
    timestamps: list[float] = []
    with av.open(str(path)) as container:
        stream = container.streams.video[0]
        stream_fps = float(stream.average_rate) if stream.average_rate else None
        for frame in container.decode(stream):
            frame_count += 1
            if width is None:
                width = int(frame.width)
                height = int(frame.height)
            if frame.pts is not None:
                timestamps.append(float(frame.pts * stream.time_base))

    timestamp_monotonic = bool(
        len(timestamps) == frame_count
        and (
            len(timestamps) < 2
            or bool((np.diff(np.asarray(timestamps, dtype=np.float64)) > 0).all())
        )
    )
    return {
        "path": str(path),
        "frames": frame_count,
        "width": width,
        "height": height,
        "fps": stream_fps,
        "timestamp_monotonic": timestamp_monotonic,
        "first_timestamp": timestamps[0] if timestamps else None,
        "last_timestamp": timestamps[-1] if timestamps else None,
    }


def checked_video_report(
    path: Path,
    cache: dict[Path, dict[str, Any]],
) -> dict[str, Any]:
    if path not in cache:
        if not path.is_file():
            cache[path] = {
                "path": str(path),
                "error": "file does not exist",
                "frames": 0,
                "width": None,
                "height": None,
                "fps": None,
                "timestamp_monotonic": False,
            }
        else:
            try:
                cache[path] = decode_video(path)
            except Exception as exc:
                cache[path] = {
                    "path": str(path),
                    "error": f"{type(exc).__name__}: {exc}",
                    "frames": 0,
                    "width": None,
                    "height": None,
                    "fps": None,
                    "timestamp_monotonic": False,
                }
    return cache[path]


def make_video_path(
    root: Path,
    template: str,
    video_key: str,
    episode_row: pd.Series,
) -> Path:
    prefix = f"videos/{video_key}"
    return root / template.format(
        video_key=video_key,
        chunk_index=int(episode_row[f"{prefix}/chunk_index"]),
        file_index=int(episode_row[f"{prefix}/file_index"]),
    )


def validate_args(args: argparse.Namespace) -> None:
    if args.expected_fps <= 0 or args.minimum_effective_fps <= 0:
        raise ValueError("FPS values must be positive.")
    if args.expected_width <= 0 or args.expected_height <= 0:
        raise ValueError("Expected video dimensions must be positive.")
    if args.minimum_vector_range < 0:
        raise ValueError("minimum-vector-range cannot be negative.")


def run_qa(args: argparse.Namespace) -> dict[str, Any]:
    validate_args(args)
    root = args.root.resolve()
    info_path = root / "meta/info.json"
    if not info_path.is_file():
        raise FileNotFoundError(f"Missing dataset metadata: {info_path}")

    info = json.loads(info_path.read_text(encoding="utf-8"))
    episodes_df = read_parquet_tree(root / "meta/episodes")
    data_df = read_parquet_tree(root / "data")
    episodes_df = episodes_df.sort_values("episode_index").reset_index(drop=True)
    data_df = data_df.sort_values("index").reset_index(drop=True)

    global_checks: list[dict[str, Any]] = []
    declared_episodes = int(info.get("total_episodes", -1))
    declared_frames = int(info.get("total_frames", -1))
    metadata_episode_indices = episodes_df["episode_index"].astype(int).to_numpy()
    expected_episode_indices = np.arange(len(episodes_df), dtype=np.int64)
    add_check(
        global_checks,
        "episode_metadata_count",
        declared_episodes == len(episodes_df) and declared_episodes > 0,
        {"declared": declared_episodes, "rows": len(episodes_df)},
    )
    add_check(
        global_checks,
        "contiguous_episode_indices",
        np.array_equal(metadata_episode_indices, expected_episode_indices),
        {
            "actual": metadata_episode_indices.tolist(),
            "expected": expected_episode_indices.tolist(),
        },
    )
    add_check(
        global_checks,
        "frame_count",
        declared_frames == len(data_df),
        {"declared": declared_frames, "rows": len(data_df)},
    )
    add_check(
        global_checks,
        "metadata_fps",
        math.isclose(float(info.get("fps", -1)), args.expected_fps),
        {"declared": info.get("fps"), "expected": args.expected_fps},
    )

    features = info.get("features", {})
    required_features = set(VIDEO_KEYS + VECTOR_KEYS)
    add_check(
        global_checks,
        "required_features",
        required_features.issubset(features),
        {"required": sorted(required_features), "actual": sorted(features)},
    )

    global_indices = data_df["index"].astype(np.int64).to_numpy()
    add_check(
        global_checks,
        "contiguous_global_indices",
        np.array_equal(global_indices, np.arange(len(data_df), dtype=np.int64)),
        {
            "first": int(global_indices[0]) if len(global_indices) else None,
            "last": int(global_indices[-1]) if len(global_indices) else None,
        },
    )

    task_index_by_text: dict[str, int] = {}
    task_path = root / "meta/tasks.parquet"
    if task_path.is_file():
        task_df = pd.read_parquet(task_path)
        for task_text, row in task_df.iterrows():
            task_index_by_text[str(task_text)] = int(row["task_index"])
    expected_task_index = task_index_by_text.get(args.expected_task)
    add_check(
        global_checks,
        "expected_task_registered",
        expected_task_index is not None,
        {
            "expected_task": args.expected_task,
            "registered_tasks": task_index_by_text,
        },
    )

    video_cache: dict[Path, dict[str, Any]] = {}
    file_usage: dict[tuple[str, Path], list[dict[str, int]]] = defaultdict(list)
    episode_reports: list[dict[str, Any]] = []
    video_template = str(
        info.get(
            "video_path",
            "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
        )
    )

    for _, episode_row in episodes_df.iterrows():
        episode_index = int(episode_row["episode_index"])
        episode_checks: list[dict[str, Any]] = []
        episode_data = data_df[data_df["episode_index"].astype(int) == episode_index]
        episode_data = episode_data.sort_values("frame_index").reset_index(drop=True)
        declared_length = int(episode_row["length"])

        add_check(
            episode_checks,
            "data_length",
            len(episode_data) == declared_length and declared_length > 0,
            {"declared": declared_length, "rows": len(episode_data)},
        )

        dataset_from = int(episode_row["dataset_from_index"])
        dataset_to = int(episode_row["dataset_to_index"])
        expected_global_indices = np.arange(dataset_from, dataset_to, dtype=np.int64)
        actual_global_indices = episode_data["index"].astype(np.int64).to_numpy()
        frame_indices = episode_data["frame_index"].astype(np.int64).to_numpy()
        add_check(
            episode_checks,
            "indices",
            bool(
                dataset_to - dataset_from == declared_length
                and np.array_equal(actual_global_indices, expected_global_indices)
                and np.array_equal(
                    frame_indices, np.arange(declared_length, dtype=np.int64)
                )
            ),
            {
                "dataset_from": dataset_from,
                "dataset_to": dataset_to,
                "first_global_index": (
                    int(actual_global_indices[0]) if len(actual_global_indices) else None
                ),
                "last_global_index": (
                    int(actual_global_indices[-1]) if len(actual_global_indices) else None
                ),
            },
        )

        timestamps = episode_data["timestamp"].astype(np.float64).to_numpy()
        finite_timestamps = bool(np.isfinite(timestamps).all())
        timestamp_deltas = np.diff(timestamps)
        timestamp_monotonic = bool(
            finite_timestamps
            and len(timestamps) == declared_length
            and (len(timestamps) < 2 or (timestamp_deltas > 0).all())
        )
        duration_s = (
            float(timestamps[-1] - timestamps[0]) if len(timestamps) >= 2 else 0.0
        )
        effective_fps = (
            float((len(timestamps) - 1) / duration_s) if duration_s > 0 else 0.0
        )
        expected_timestamps = frame_indices.astype(np.float64) / args.expected_fps
        cadence_ok = bool(
            len(timestamps) == len(expected_timestamps)
            and np.allclose(
                timestamps,
                expected_timestamps,
                atol=max(1e-5, 1 / args.expected_fps / 100),
                rtol=0,
            )
        )
        add_check(
            episode_checks,
            "timestamps",
            timestamp_monotonic
            and cadence_ok
            and effective_fps >= args.minimum_effective_fps,
            {
                "finite": finite_timestamps,
                "monotonic": timestamp_monotonic,
                "cadence_matches_frame_index": cadence_ok,
                "duration_s": duration_s,
                "effective_fps": effective_fps,
                "minimum_effective_fps": args.minimum_effective_fps,
            },
        )

        episode_tasks = normalize_tasks(episode_row.get("tasks"))
        actual_task_indices = (
            sorted(episode_data["task_index"].astype(int).unique().tolist())
            if "task_index" in episode_data
            else []
        )
        add_check(
            episode_checks,
            "task",
            episode_tasks == [args.expected_task]
            and expected_task_index is not None
            and actual_task_indices == [expected_task_index],
            {
                "metadata_tasks": episode_tasks,
                "data_task_indices": actual_task_indices,
                "expected_task_index": expected_task_index,
            },
        )

        vector_reports: dict[str, Any] = {}
        for key in VECTOR_KEYS:
            try:
                values = stack_vector(episode_data[key])
                ranges = (
                    np.ptp(values, axis=0).astype(float).tolist()
                    if values.ndim == 2 and values.shape[1] == 7
                    else None
                )
                finite = bool(np.isfinite(values).all())
                shape_ok = values.shape == (declared_length, 7)
                varied = bool(
                    ranges is not None
                    and max(ranges, default=0.0) >= args.minimum_vector_range
                )
                report = {
                    "shape": list(values.shape),
                    "finite": finite,
                    "ranges": ranges,
                    "varied": varied,
                    "minimum_vector_range": args.minimum_vector_range,
                }
                passed = finite and shape_ok and varied
            except Exception as exc:
                report = {"error": f"{type(exc).__name__}: {exc}"}
                passed = False
            vector_reports[key] = report
            add_check(episode_checks, f"{key}_quality", passed, report)

        video_reports: dict[str, Any] = {}
        for video_key in VIDEO_KEYS:
            try:
                path = make_video_path(root, video_template, video_key, episode_row)
                physical = checked_video_report(path, video_cache)
                prefix = f"videos/{video_key}"
                from_timestamp = float(episode_row[f"{prefix}/from_timestamp"])
                to_timestamp = float(episode_row[f"{prefix}/to_timestamp"])
                segment_from_frame = round(from_timestamp * args.expected_fps)
                segment_to_frame = round(to_timestamp * args.expected_fps)
                segment_frames = segment_to_frame - segment_from_frame
                physical_ok = bool(
                    "error" not in physical
                    and physical["frames"] > 0
                    and physical["width"] == args.expected_width
                    and physical["height"] == args.expected_height
                    and physical["timestamp_monotonic"]
                    and physical["fps"] is not None
                    and math.isclose(
                        float(physical["fps"]), args.expected_fps, rel_tol=0, abs_tol=0.01
                    )
                )
                interval_ok = bool(
                    segment_from_frame >= 0
                    and segment_to_frame <= int(physical["frames"])
                    and segment_frames == declared_length
                )
                report = {
                    "path": str(path),
                    "physical_frames": int(physical["frames"]),
                    "width": physical["width"],
                    "height": physical["height"],
                    "fps": physical["fps"],
                    "timestamp_monotonic": physical["timestamp_monotonic"],
                    "from_timestamp": from_timestamp,
                    "to_timestamp": to_timestamp,
                    "segment_from_frame": segment_from_frame,
                    "segment_to_frame": segment_to_frame,
                    "segment_frames": segment_frames,
                    "expected_episode_frames": declared_length,
                    "physical_ok": physical_ok,
                    "interval_ok": interval_ok,
                }
                if "error" in physical:
                    report["error"] = physical["error"]
                passed = physical_ok and interval_ok
                file_usage[(video_key, path)].append(
                    {
                        "episode_index": episode_index,
                        "from_frame": segment_from_frame,
                        "to_frame": segment_to_frame,
                    }
                )
            except Exception as exc:
                report = {"error": f"{type(exc).__name__}: {exc}"}
                passed = False
            video_reports[video_key] = report
            add_check(episode_checks, f"{video_key}_video", passed, report)

        episode_reports.append(
            {
                "episode_index": episode_index,
                "passed": all(check["passed"] for check in episode_checks),
                "checks": episode_checks,
                "vectors": vector_reports,
                "videos": video_reports,
            }
        )

    physical_video_reports: list[dict[str, Any]] = []
    for (video_key, path), intervals in sorted(
        file_usage.items(), key=lambda item: (item[0][0], str(item[0][1]))
    ):
        intervals = sorted(intervals, key=lambda item: item["from_frame"])
        physical = video_cache[path]
        expected_from = 0
        contiguous = True
        for interval in intervals:
            if interval["from_frame"] != expected_from:
                contiguous = False
            expected_from = interval["to_frame"]
        covers_file = bool(
            contiguous
            and "error" not in physical
            and expected_from == int(physical["frames"])
        )
        report = {
            **physical,
            "video_key": video_key,
            "episode_intervals": intervals,
            "episode_intervals_contiguous_and_cover_file": covers_file,
        }
        physical_video_reports.append(report)
        add_check(
            global_checks,
            f"physical_video_coverage:{video_key}:{path.name}",
            covers_file,
            {
                "path": str(path),
                "decoded_frames": physical["frames"],
                "covered_to_frame": expected_from,
                "intervals_contiguous": contiguous,
            },
        )

    referenced_paths = {path.resolve() for _, path in file_usage}
    discovered_paths = {path.resolve() for path in root.rglob("*.mp4")}
    extra_video_paths = sorted(str(path) for path in discovered_paths - referenced_paths)
    add_check(
        global_checks,
        "no_unreferenced_mp4_files",
        not extra_video_paths,
        {"unreferenced": extra_video_paths},
    )

    failed_episode_indices = [
        int(report["episode_index"])
        for report in episode_reports
        if not report["passed"]
    ]
    passed_episode_indices = [
        int(report["episode_index"])
        for report in episode_reports
        if report["passed"]
    ]
    passed = bool(
        all(check["passed"] for check in global_checks)
        and not failed_episode_indices
    )
    return {
        "passed": passed,
        "repo_id": args.repo_id,
        "root": str(root),
        "summary": {
            "episode_count": len(episode_reports),
            "passed_episode_count": len(episode_reports) - len(failed_episode_indices),
            "failed_episode_count": len(failed_episode_indices),
            "passed_episode_indices": passed_episode_indices,
            "failed_episode_indices": failed_episode_indices,
            "physical_video_file_count": len(physical_video_reports),
        },
        "global_checks": global_checks,
        "episodes": episode_reports,
        "physical_videos": physical_video_reports,
        "manual_checks_required": [
            "夹爪夹住的是订合在一起的两根橙黄色提带。",
            "袋底竖直提离桌面约10厘米，并连续悬空约2秒。",
            "袋子回到标记区域，夹爪松开提带并退离。",
            "全过程无滑脱、倾倒、明显遮挡或人员进入画面。",
            "录制前后主机CAN TX计数增量为0。",
        ],
    }


def print_summary(result: dict[str, Any]) -> None:
    summary = result["summary"]
    status = "PASS" if result["passed"] else "FAIL"
    print(
        f"Dataset QA {status}: {summary['passed_episode_count']}/"
        f"{summary['episode_count']} episodes passed"
    )
    if summary["failed_episode_indices"]:
        print(f"Failed episode indices: {summary['failed_episode_indices']}")
    for report in result["episodes"]:
        failed_checks = [
            check["name"] for check in report["checks"] if not check["passed"]
        ]
        episode_status = "PASS" if report["passed"] else "FAIL"
        suffix = "" if not failed_checks else f" ({', '.join(failed_checks)})"
        print(f"  episode {report['episode_index']}: {episode_status}{suffix}")

    failed_global = [
        check["name"] for check in result["global_checks"] if not check["passed"]
    ]
    if failed_global:
        print(f"Failed dataset-wide checks: {', '.join(failed_global)}")


def main() -> int:
    args = parse_args()
    try:
        result = run_qa(args)
    except Exception as exc:
        print(f"Dataset QA could not run: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    payload = json.dumps(result, ensure_ascii=False, indent=2)
    if args.print_json:
        print(payload)
    else:
        print_summary(result)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
        print(f"JSON report: {args.output.resolve()}")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
