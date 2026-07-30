"""Regression tests for the multi-episode Piper dataset QA tool."""

from __future__ import annotations

import json
import tempfile
import unittest
from argparse import Namespace
from fractions import Fraction
from pathlib import Path

import av
import numpy as np
import pandas as pd

from scripts.verify_piper_dataset import DEFAULT_TASK, run_qa


class BatchDatasetQATest(unittest.TestCase):
    def _write_video(self, path: Path, frame_count: int = 9) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with av.open(str(path), "w") as container:
            stream = container.add_stream("mpeg4", rate=30)
            stream.width = 96
            stream.height = 64
            stream.pix_fmt = "yuv420p"
            stream.time_base = Fraction(1, 30)
            for frame_index in range(frame_count):
                image = np.full(
                    (64, 96, 3), frame_index * 10, dtype=np.uint8
                )
                frame = av.VideoFrame.from_ndarray(image, format="rgb24")
                frame.pts = frame_index
                frame.time_base = Fraction(1, 30)
                container.mux(stream.encode(frame))
            container.mux(stream.encode())

    def _make_dataset(self, root: Path) -> Namespace:
        info = {
            "fps": 30,
            "total_episodes": 2,
            "total_frames": 9,
            "features": {
                "observation.images.front": {
                    "dtype": "video",
                    "shape": [64, 96, 3],
                },
                "observation.images.wrist": {
                    "dtype": "video",
                    "shape": [64, 96, 3],
                },
                "observation.state": {"dtype": "float32", "shape": [7]},
                "action": {"dtype": "float32", "shape": [7]},
            },
            "video_path": (
                "videos/{video_key}/chunk-{chunk_index:03d}/"
                "file-{file_index:03d}.mp4"
            ),
        }
        (root / "meta").mkdir(parents=True)
        (root / "meta/info.json").write_text(
            json.dumps(info, ensure_ascii=False), encoding="utf-8"
        )
        pd.DataFrame(
            {"task_index": [0]},
            index=pd.Index([DEFAULT_TASK], name="task"),
        ).to_parquet(root / "meta/tasks.parquet")

        episodes = []
        global_start = 0
        video_start = 0
        for episode_index, length in enumerate((4, 5)):
            row = {
                "episode_index": episode_index,
                "tasks": [DEFAULT_TASK],
                "length": length,
                "dataset_from_index": global_start,
                "dataset_to_index": global_start + length,
            }
            for video_key in (
                "observation.images.front",
                "observation.images.wrist",
            ):
                prefix = f"videos/{video_key}"
                row[f"{prefix}/chunk_index"] = 0
                row[f"{prefix}/file_index"] = 0
                row[f"{prefix}/from_timestamp"] = video_start / 30
                row[f"{prefix}/to_timestamp"] = (video_start + length) / 30
            episodes.append(row)
            global_start += length
            video_start += length
        episode_path = root / "meta/episodes/chunk-000/file-000.parquet"
        episode_path.parent.mkdir(parents=True)
        pd.DataFrame(episodes).to_parquet(episode_path)

        data_rows = []
        global_index = 0
        for episode_index, length in enumerate((4, 5)):
            for frame_index in range(length):
                vector = np.arange(7, dtype=np.float32) + frame_index
                data_rows.append(
                    {
                        "index": global_index,
                        "episode_index": episode_index,
                        "frame_index": frame_index,
                        "timestamp": frame_index / 30,
                        "task_index": 0,
                        "observation.state": vector,
                        "action": vector + 0.25,
                    }
                )
                global_index += 1
        data_path = root / "data/chunk-000/file-000.parquet"
        data_path.parent.mkdir(parents=True)
        pd.DataFrame(data_rows).to_parquet(data_path)

        for video_key in (
            "observation.images.front",
            "observation.images.wrist",
        ):
            self._write_video(
                root
                / f"videos/{video_key}/chunk-000/file-000.mp4"
            )

        return Namespace(
            repo_id="local/test-batch-qa",
            root=root,
            expected_task=DEFAULT_TASK,
            expected_fps=30.0,
            minimum_effective_fps=28.0,
            expected_width=96,
            expected_height=64,
            minimum_vector_range=0.5,
            output=None,
            print_json=False,
        )

    def test_two_episodes_sharing_each_mp4_pass(self) -> None:
        with tempfile.TemporaryDirectory(prefix="batch_qa_test_") as temp:
            args = self._make_dataset(Path(temp))
            result = run_qa(args)
        self.assertTrue(result["passed"])
        self.assertEqual(result["summary"]["episode_count"], 2)
        self.assertEqual(result["summary"]["passed_episode_indices"], [0, 1])
        self.assertEqual(result["summary"]["failed_episode_indices"], [])

    def test_one_bad_episode_video_interval_is_reported(self) -> None:
        with tempfile.TemporaryDirectory(prefix="batch_qa_test_") as temp:
            root = Path(temp)
            args = self._make_dataset(root)
            episode_path = root / "meta/episodes/chunk-000/file-000.parquet"
            episodes = pd.read_parquet(episode_path)
            episodes.loc[
                episodes["episode_index"] == 0,
                "videos/observation.images.wrist/to_timestamp",
            ] = 3 / 30
            episodes.to_parquet(episode_path)
            result = run_qa(args)
        self.assertFalse(result["passed"])
        self.assertEqual(result["summary"]["passed_episode_indices"], [1])
        self.assertEqual(result["summary"]["failed_episode_indices"], [0])


if __name__ == "__main__":
    unittest.main()
