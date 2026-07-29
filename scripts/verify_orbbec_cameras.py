#!/usr/bin/env python3

import argparse
import json
import statistics
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from threading import Event, Thread
from typing import Any

import numpy as np
from PIL import Image

from lerobot_camera_orbbec import OrbbecCamera, OrbbecCameraConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify configured Orbbec cameras concurrently without changing firmware."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("/home/ubuntu22/Piper-VLA/config/cameras.json"),
    )
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--warmup-s", type=float)
    parser.add_argument("--include-depth", action="store_true")
    parser.add_argument("--minimum-fps", type=float, default=28.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--snapshot-dir", type=Path)
    return parser.parse_args()


def load_config(path: Path, include_depth: bool, warmup_override: float | None) -> tuple[dict, dict]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    capture = raw["capture"]
    warmup_s = capture["warmup_s"] if warmup_override is None else warmup_override
    configs: dict[str, OrbbecCameraConfig] = {}
    for logical_name, camera in raw["cameras"].items():
        configs[logical_name] = OrbbecCameraConfig(
            serial_number=camera["serial_number"],
            width=capture["width"],
            height=capture["height"],
            fps=capture["fps"],
            color_mode=capture["color_mode"],
            use_depth=include_depth,
            timeout_ms=capture["timeout_ms"],
            warmup_s=warmup_s,
        )
    return raw, configs


def stats_delta(after: dict[str, Any], before: dict[str, Any], key: str) -> int:
    return int(after[key]) - int(before[key])


def consume_camera(
    name: str,
    camera: OrbbecCamera,
    duration_s: float,
    include_depth: bool,
    stop_event: Event,
) -> dict[str, Any]:
    end = time.perf_counter() + duration_s
    arrivals: list[float] = []
    indices: list[int] = []
    hardware_timestamps: list[int] = []
    timeouts = 0
    bad_color_shapes = 0
    bad_depth_shapes = 0
    color_samples: list[float] = []
    depth_samples: list[float] = []

    while time.perf_counter() < end and not stop_event.is_set():
        try:
            image = camera.async_read(timeout_ms=1000)
            arrival = time.perf_counter()
            metadata = camera.get_last_frame_metadata()
        except TimeoutError:
            timeouts += 1
            continue

        if image.shape != (camera.height, camera.width, 3) or image.dtype != np.uint8:
            bad_color_shapes += 1
        elif len(color_samples) < 30:
            color_samples.append(float(image[::32, ::32].mean()))

        if include_depth:
            depth = camera.async_read_depth(timeout_ms=1000)
            if depth.shape != (camera.height, camera.width, 1) or depth.dtype != np.uint16:
                bad_depth_shapes += 1
            elif len(depth_samples) < 30:
                nonzero = depth[depth > 0]
                if nonzero.size:
                    depth_samples.append(float(np.median(nonzero)))

        arrivals.append(arrival)
        indices.append(int(metadata["frame_index"]))
        hardware_timestamps.append(int(metadata["hardware_timestamp_us"]))

    span = arrivals[-1] - arrivals[0] if len(arrivals) > 1 else 0.0
    fps = (len(arrivals) - 1) / span if span > 0 else 0.0
    metadata_duplicates = sum(b == a for a, b in zip(indices, indices[1:]))
    metadata_regressions = sum(b < a for a, b in zip(indices, indices[1:]))
    timestamp_regressions = sum(
        b <= a for a, b in zip(hardware_timestamps, hardware_timestamps[1:])
    )
    return {
        "logical_name": name,
        "consumer_frames": len(arrivals),
        "consumer_fps": fps,
        "consumer_timeouts": timeouts,
        "bad_color_shapes": bad_color_shapes,
        "bad_depth_shapes": bad_depth_shapes,
        "metadata_duplicate_indices": metadata_duplicates,
        "metadata_index_regressions": metadata_regressions,
        "metadata_timestamp_regressions": timestamp_regressions,
        "sample_color_mean": statistics.mean(color_samples) if color_samples else None,
        "sample_depth_median_raw": statistics.mean(depth_samples) if depth_samples else None,
    }


def main() -> int:
    args = parse_args()
    if args.duration_s <= 0:
        raise ValueError("--duration-s must be positive")

    raw_config, camera_configs = load_config(args.config, args.include_depth, args.warmup_s)
    configured_serials = {cfg.serial_number for cfg in camera_configs.values()}
    enumeration = OrbbecCamera.find_cameras()
    enumerated_serials = {str(item["serial_number"]) for item in enumeration}

    cameras = {name: OrbbecCamera(config) for name, config in camera_configs.items()}
    connected: list[OrbbecCamera] = []
    started_at = datetime.now().astimezone().isoformat()
    result: dict[str, Any] = {
        "started_at": started_at,
        "config_path": str(args.config.resolve()),
        "mode": "rgb_depth_diagnostic" if args.include_depth else "rgb_only",
        "duration_s": args.duration_s,
        "configured_serials": sorted(configured_serials),
        "enumerated_devices": enumeration,
        "cameras": {},
        "passed": False,
        "failures": [],
    }

    if enumerated_serials != configured_serials:
        result["failures"].append(
            f"Configured serials {sorted(configured_serials)} do not exactly match "
            f"enumerated serials {sorted(enumerated_serials)}."
        )

    try:
        with ThreadPoolExecutor(max_workers=len(cameras)) as executor:
            futures = {name: executor.submit(camera.connect) for name, camera in cameras.items()}
            for name, future in futures.items():
                future.result()
                connected.append(cameras[name])

        before = {name: camera.get_health_stats() for name, camera in cameras.items()}
        stop_event = Event()
        consumer_results: dict[str, dict[str, Any]] = {}
        threads: list[Thread] = []

        def run_consumer(logical_name: str, camera: OrbbecCamera) -> None:
            consumer_results[logical_name] = consume_camera(
                logical_name,
                camera,
                args.duration_s,
                args.include_depth,
                stop_event,
            )

        for name, camera in cameras.items():
            thread = Thread(target=run_consumer, args=(name, camera), daemon=True)
            threads.append(thread)
            thread.start()
        for thread in threads:
            thread.join(timeout=args.duration_s + 5)
        if any(thread.is_alive() for thread in threads):
            stop_event.set()
            result["failures"].append("One or more camera consumer threads did not stop.")

        after = {name: camera.get_health_stats() for name, camera in cameras.items()}

        for name, camera in cameras.items():
            consumer = consumer_results[name]
            frames = stats_delta(after[name], before[name], "frames_received")
            elapsed = args.duration_s
            capture_fps = frames / elapsed
            camera_result = {
                **consumer,
                "serial_number": camera.serial_number,
                "physical_role": raw_config["cameras"][name]["physical_role"],
                "profile": {
                    "width": camera.width,
                    "height": camera.height,
                    "fps": camera.fps,
                    "format": "RGB",
                    "depth_enabled": camera.use_depth,
                },
                "capture_frames": frames,
                "capture_fps": capture_fps,
                "frame_index_gaps": stats_delta(
                    after[name], before[name], "frame_index_gaps"
                ),
                "duplicate_frame_indices": stats_delta(
                    after[name], before[name], "duplicate_frame_indices"
                ),
                "hardware_timestamp_regressions": stats_delta(
                    after[name], before[name], "hardware_timestamp_regressions"
                ),
                "sdk_timeouts": stats_delta(after[name], before[name], "timeouts"),
                "read_failures": stats_delta(after[name], before[name], "read_failures"),
                "bad_color_frames": stats_delta(after[name], before[name], "bad_color_frames"),
                "bad_depth_frames": stats_delta(after[name], before[name], "bad_depth_frames"),
                "latest_frame_age_ms": after[name]["latest_frame_age_ms"],
            }
            result["cameras"][name] = camera_result

            if camera_result["capture_fps"] < args.minimum_fps:
                result["failures"].append(
                    f"{name} capture FPS {camera_result['capture_fps']:.2f} "
                    f"is below {args.minimum_fps:.2f}."
                )
            for metric in (
                "frame_index_gaps",
                "duplicate_frame_indices",
                "hardware_timestamp_regressions",
                "sdk_timeouts",
                "read_failures",
                "bad_color_frames",
                "bad_depth_frames",
                "consumer_timeouts",
                "bad_color_shapes",
                "bad_depth_shapes",
                "metadata_index_regressions",
                "metadata_timestamp_regressions",
            ):
                if camera_result[metric] != 0:
                    result["failures"].append(
                        f"{name} reported {metric}={camera_result[metric]}."
                    )

        if args.snapshot_dir is not None:
            args.snapshot_dir.mkdir(parents=True, exist_ok=True)
            snapshots: dict[str, str] = {}
            for name, camera in cameras.items():
                image = camera.read_latest(max_age_ms=1000)
                path = args.snapshot_dir / f"{name}_rgb.png"
                Image.fromarray(image).save(path)
                snapshots[name] = str(path.resolve())
            result["snapshots"] = snapshots

    except Exception as exc:
        result["failures"].append(f"{type(exc).__name__}: {exc}")
    finally:
        for camera in connected:
            if camera.is_connected:
                try:
                    camera.disconnect()
                except Exception as exc:
                    result["failures"].append(
                        f"Failed to disconnect {camera.serial_number}: {type(exc).__name__}: {exc}"
                    )

    result["passed"] = not result["failures"]
    result["finished_at"] = datetime.now().astimezone().isoformat()

    rendered = json.dumps(result, indent=2, ensure_ascii=False)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
