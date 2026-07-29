#!/usr/bin/env python3

"""Serve the live Piper/Orbbec training-data inspection dashboard."""

import argparse
import copy
import html
import io
import json
import sys
import time
import webbrowser
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Condition, Event, Lock, Thread
from typing import Any

from PIL import Image

from lerobot_camera_orbbec import OrbbecCamera, OrbbecCameraConfig
from lerobot_robot_piper import PiperRobot, PiperRobotConfig
from lerobot_teleoperator_piper import (
    PiperMasterTeleoperator,
    PiperMasterTeleoperatorConfig,
)


DEFAULT_CONFIG = Path("/home/ubuntu22/Piper-VLA/config/cameras.json")
DEFAULT_PIPER_CONFIG = Path(
    "/home/ubuntu22/Piper-VLA/config/piper_native_master_slave.json"
)
PIPER_FEATURES = tuple(
    [f"joint_{index}.pos" for index in range(1, 7)] + ["gripper.pos"]
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Display all configured Orbbec RGB cameras in a local browser and "
            "show the image, follower state, master action, and capture health "
            "needed for a LeRobot training sample."
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--camera",
        action="append",
        help=(
            "Only display this logical camera name (for example, front or wrist). "
            "Repeat the option to select more than one. By default, all connected "
            "configured cameras are displayed."
        ),
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--display-fps",
        type=float,
        default=15.0,
        help="Browser refresh rate per camera; hardware still captures at the configured FPS.",
    )
    parser.add_argument("--jpeg-quality", type=int, default=85)
    parser.add_argument("--max-frame-age-ms", type=int, default=1000)
    parser.add_argument(
        "--piper-config",
        type=Path,
        default=DEFAULT_PIPER_CONFIG,
        help="Native master/slave CAN identity and units configuration.",
    )
    parser.add_argument(
        "--piper-refresh-fps",
        type=float,
        default=30.0,
        help="Passive follower-state/master-action sampling rate.",
    )
    parser.add_argument(
        "--task",
        default="未设置（仅做实时数据检查）",
        help="Task instruction shown with the prospective training sample.",
    )
    parser.add_argument(
        "--camera-only",
        action="store_true",
        help="Do not open the passive CAN readers; display camera data only.",
    )
    parser.add_argument(
        "--require-piper",
        action="store_true",
        help="Exit if the passive Piper data source cannot be connected.",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Start the viewer without opening the default browser.",
    )
    return parser.parse_args()


def read_counter(path: Path) -> int | None:
    try:
        return int(path.read_text(encoding="utf-8").strip())
    except (FileNotFoundError, OSError, ValueError):
        return None


class PiperDataSource:
    """Read follower observations and master targets without CAN transmission."""

    def __init__(self, config_path: Path, *, refresh_fps: float) -> None:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
        can_config = raw["can"]
        if raw.get("control_chain") != "native_master_slave":
            raise ValueError("Only native_master_slave Piper data is supported.")
        if tuple(raw.get("features", ())) != PIPER_FEATURES:
            raise ValueError(
                f"Piper features must be exactly {list(PIPER_FEATURES)}."
            )
        if raw.get("units") != {
            "joints": "degrees",
            "gripper": "millimeters",
        }:
            raise ValueError("Piper units must be degrees and millimeters.")
        if not raw.get("safety", {}).get("software_actuation_forbidden", False):
            raise ValueError("Piper configuration must forbid software actuation.")

        self.config_path = config_path
        self.interface = str(can_config["interface"])
        self.bitrate = int(can_config["bitrate"])
        self.expected_adapter_serial = str(can_config["expected_adapter_serial"])
        self.refresh_fps = refresh_fps
        self._tx_path = (
            Path("/sys/class/net")
            / self.interface
            / "statistics"
            / "tx_packets"
        )
        self._tx_baseline: int | None = None
        self._robot: PiperRobot | None = None
        self._teleop: PiperMasterTeleoperator | None = None
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._lock = Lock()
        self._sample_times: deque[float] = deque(maxlen=90)
        self._state: dict[str, Any] = {
            "enabled": True,
            "connected": False,
            "starting": False,
            "sample_count": 0,
            "sample_fps": None,
            "latest_sample_age_ms": None,
            "observation": None,
            "action": None,
            "delta": None,
            "health": {},
            "error": None,
        }

    def _interface_preflight(self) -> None:
        interface_path = Path("/sys/class/net") / self.interface
        if not interface_path.exists():
            raise ConnectionError(f"CAN 接口 {self.interface} 不存在。")
        operstate = (interface_path / "operstate").read_text(
            encoding="utf-8"
        ).strip()
        if operstate != "up":
            raise ConnectionError(
                f"CAN 接口 {self.interface} 当前为 {operstate.upper()}；"
                "网页不会自动修改接口状态。"
            )

    def start(self) -> None:
        with self._lock:
            self._state["starting"] = True
            self._state["error"] = None
        try:
            self._interface_preflight()
            self._tx_baseline = read_counter(self._tx_path)
            robot_config = PiperRobotConfig(
                id="dashboard_follower",
                can_interface=self.interface,
                expected_adapter_serial=self.expected_adapter_serial,
                cameras={},
            )
            teleop_config = PiperMasterTeleoperatorConfig(
                id="dashboard_master",
                can_interface=self.interface,
                expected_adapter_serial=self.expected_adapter_serial,
            )
            self._robot = PiperRobot(robot_config)
            self._teleop = PiperMasterTeleoperator(teleop_config)
            self._robot.connect(calibrate=False)
            self._teleop.connect(calibrate=False)

            with self._lock:
                self._state["connected"] = True
                self._state["starting"] = False
            self._stop_event.clear()
            self._thread = Thread(
                target=self._poll_loop,
                name="piper_dataset_dashboard_reader",
                daemon=True,
            )
            self._thread.start()
        except Exception as exc:
            self._disconnect_readers()
            with self._lock:
                self._state["connected"] = False
                self._state["starting"] = False
                self._state["error"] = f"{type(exc).__name__}: {exc}"
            raise

    def _poll_loop(self) -> None:
        period_s = 1.0 / self.refresh_fps
        next_sample_at = time.perf_counter()
        while not self._stop_event.is_set():
            try:
                assert self._robot is not None
                assert self._teleop is not None
                observation = {
                    key: float(value)
                    for key, value in self._robot.get_observation().items()
                    if key in PIPER_FEATURES
                }
                action = {
                    key: float(value)
                    for key, value in self._teleop.get_action().items()
                }
                if tuple(observation) != PIPER_FEATURES:
                    raise ValueError("Follower observation feature order is invalid.")
                if tuple(action) != PIPER_FEATURES:
                    raise ValueError("Master action feature order is invalid.")

                now = time.perf_counter()
                delta = {
                    key: action[key] - observation[key] for key in PIPER_FEATURES
                }
                health = self._teleop.get_health_stats()
                with self._lock:
                    self._sample_times.append(now)
                    sample_fps = None
                    if len(self._sample_times) >= 2:
                        elapsed = self._sample_times[-1] - self._sample_times[0]
                        if elapsed > 0:
                            sample_fps = (
                                len(self._sample_times) - 1
                            ) / elapsed
                    self._state.update(
                        {
                            "sample_count": self._state["sample_count"] + 1,
                            "sample_fps": sample_fps,
                            "latest_sample_monotonic": now,
                            "observation": observation,
                            "action": action,
                            "delta": delta,
                            "health": health,
                            "error": None,
                        }
                    )
            except Exception as exc:
                with self._lock:
                    self._state["error"] = f"{type(exc).__name__}: {exc}"

            next_sample_at += period_s
            delay = next_sample_at - time.perf_counter()
            if delay <= 0:
                next_sample_at = time.perf_counter()
                continue
            self._stop_event.wait(delay)

    def status(self) -> dict[str, Any]:
        with self._lock:
            state = copy.deepcopy(self._state)
        latest = state.pop("latest_sample_monotonic", None)
        state["latest_sample_age_ms"] = (
            None if latest is None else (time.perf_counter() - latest) * 1000
        )
        tx_now = read_counter(self._tx_path)
        state["can"] = {
            "interface": self.interface,
            "bitrate": self.bitrate,
            "expected_adapter_serial": self.expected_adapter_serial,
            "tx_packets_at_start": self._tx_baseline,
            "tx_packets_now": tx_now,
            "tx_packets_delta": (
                None
                if self._tx_baseline is None or tx_now is None
                else tx_now - self._tx_baseline
            ),
            "receive_only": True,
        }
        return state

    def _disconnect_readers(self) -> None:
        if self._teleop is not None:
            self._teleop.disconnect()
        if self._robot is not None and self._robot.is_connected:
            self._robot.disconnect()
        self._teleop = None
        self._robot = None

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._disconnect_readers()
        with self._lock:
            self._state["connected"] = False


class StreamBuffer:
    def __init__(self) -> None:
        self.condition = Condition()
        self.jpeg: bytes | None = None
        self.sequence = 0
        self.encoded_frames = 0
        self.last_encode_monotonic: float | None = None
        self.error: str | None = None

    def publish(self, jpeg: bytes) -> None:
        with self.condition:
            self.jpeg = jpeg
            self.sequence += 1
            self.encoded_frames += 1
            self.last_encode_monotonic = time.perf_counter()
            self.error = None
            self.condition.notify_all()

    def report_error(self, error: BaseException) -> None:
        with self.condition:
            self.error = f"{type(error).__name__}: {error}"
            self.condition.notify_all()

    def status(self) -> dict[str, Any]:
        with self.condition:
            age_ms = (
                None
                if self.last_encode_monotonic is None
                else (time.perf_counter() - self.last_encode_monotonic) * 1000
            )
            return {
                "encoded_frames": self.encoded_frames,
                "latest_jpeg_age_ms": age_ms,
                "error": self.error,
            }


class ViewerState:
    def __init__(
        self,
        raw_config: dict[str, Any],
        cameras: dict[str, OrbbecCamera],
        *,
        display_fps: float,
        jpeg_quality: int,
        max_frame_age_ms: int,
        task: str,
        piper: PiperDataSource | None,
    ) -> None:
        self.raw_config = raw_config
        self.cameras = cameras
        self.display_fps = display_fps
        self.jpeg_quality = jpeg_quality
        self.max_frame_age_ms = max_frame_age_ms
        self.task = task
        self.piper = piper
        self.started_monotonic = time.perf_counter()
        self.stop_event = Event()
        self.buffers = {name: StreamBuffer() for name in cameras}
        self.encoder_threads: list[Thread] = []

    def start_encoders(self) -> None:
        for name, camera in self.cameras.items():
            thread = Thread(
                target=self._encode_loop,
                args=(name, camera),
                name=f"{name}_browser_encoder",
                daemon=True,
            )
            self.encoder_threads.append(thread)
            thread.start()

    def _encode_loop(self, name: str, camera: OrbbecCamera) -> None:
        period_s = 1.0 / self.display_fps
        next_frame_at = time.perf_counter()
        stream = self.buffers[name]

        while not self.stop_event.is_set():
            try:
                frame = camera.read_latest(max_age_ms=self.max_frame_age_ms)
                output = io.BytesIO()
                Image.fromarray(frame).save(
                    output,
                    format="JPEG",
                    quality=self.jpeg_quality,
                    optimize=False,
                )
                stream.publish(output.getvalue())
            except Exception as exc:
                stream.report_error(exc)

            next_frame_at += period_s
            delay = next_frame_at - time.perf_counter()
            if delay <= 0:
                next_frame_at = time.perf_counter()
                continue
            self.stop_event.wait(delay)

    def wait_until_ready(self, timeout_s: float = 5.0) -> None:
        deadline = time.perf_counter() + timeout_s
        for name, stream in self.buffers.items():
            with stream.condition:
                while stream.jpeg is None and stream.error is None:
                    remaining = deadline - time.perf_counter()
                    if remaining <= 0:
                        raise TimeoutError(f"{name} did not produce a display frame.")
                    stream.condition.wait(timeout=remaining)
                if stream.jpeg is None:
                    raise RuntimeError(f"{name} display encoder failed: {stream.error}")

    def status(self) -> dict[str, Any]:
        cameras: dict[str, Any] = {}
        for name, camera in self.cameras.items():
            try:
                health = camera.get_health_stats()
            except Exception as exc:
                health = {"connected": False, "status_error": f"{type(exc).__name__}: {exc}"}

            camera_config = self.raw_config["cameras"][name]
            cameras[name] = {
                "serial_number": camera.serial_number,
                "physical_role": camera_config["physical_role"],
                "profile": {
                    "width": camera.width,
                    "height": camera.height,
                    "capture_fps": camera.fps,
                    "display_fps_limit": self.display_fps,
                    "color_mode": str(camera.color_mode.value),
                    "depth_enabled": camera.use_depth,
                },
                "viewer": self.buffers[name].status(),
                "capture": health,
            }

        return {
            "uptime_s": time.perf_counter() - self.started_monotonic,
            "task": self.task,
            "cameras": cameras,
            "piper": (
                {
                    "enabled": False,
                    "connected": False,
                    "error": "已使用 --camera-only，未打开 CAN 被动读取器。",
                }
                if self.piper is None
                else self.piper.status()
            ),
            "training_sample": {
                "observation_image_features": [
                    f"observation.images.{name}" for name in self.cameras
                ],
                "observation_state_features": list(PIPER_FEATURES),
                "action_features": list(PIPER_FEATURES),
                "task_feature": "task",
                "time_features": ["timestamp", "frame_index", "episode_index"],
            },
        }

    def stop(self) -> None:
        self.stop_event.set()
        for stream in self.buffers.values():
            with stream.condition:
                stream.condition.notify_all()
        for thread in self.encoder_threads:
            thread.join(timeout=2.0)
        if self.piper is not None:
            self.piper.stop()


def build_index_html(state: ViewerState) -> bytes:
    cards: list[str] = []
    camera_dom_ids: dict[str, str] = {}
    for index, (name, camera) in enumerate(state.cameras.items()):
        dom_id = f"camera-{index}"
        camera_dom_ids[name] = dom_id
        role = state.raw_config["cameras"][name]["physical_role"]
        cards.append(
            f"""
            <section class="camera-card" id="{dom_id}">
              <header>
                <div>
                  <h2>{html.escape(name)}</h2>
                  <p>{html.escape(role)} · {html.escape(camera.serial_number)}</p>
                </div>
                <span class="badge" data-field="state">正在连接</span>
              </header>
              <div class="image-shell">
                <img src="/stream/{html.escape(name)}.mjpg"
                     alt="{html.escape(name)} 实时画面">
              </div>
              <dl>
                <div><dt>采集帧率</dt><dd data-field="fps">--</dd></div>
                <div><dt>累计帧数</dt><dd data-field="frames">--</dd></div>
                <div><dt>图像年龄</dt><dd data-field="age">--</dd></div>
                <div><dt>异常统计</dt><dd data-field="errors">--</dd></div>
              </dl>
            </section>
            """
        )

    camera_dom_json = json.dumps(camera_dom_ids, ensure_ascii=False)
    feature_rows: list[str] = []
    for index, feature in enumerate(PIPER_FEATURES):
        label = f"关节 J{index + 1}" if index < 6 else "夹爪"
        unit = "°" if index < 6 else "mm"
        feature_rows.append(
            f"""
            <tr data-feature="{html.escape(feature)}">
              <th>{label}<small>{html.escape(feature)}</small></th>
              <td data-value="observation">--</td>
              <td data-value="action">--</td>
              <td data-value="delta">--</td>
              <td>{unit}</td>
            </tr>
            """
        )
    image_feature_chips = "".join(
        f"<span>observation.images.{html.escape(name)}</span>"
        for name in state.cameras
    )
    task_text = html.escape(state.task)
    page = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Piper-VLA 训练数据实时检查</title>
  <style>
    :root {{
      color-scheme: dark;
      font-family: Inter, "Noto Sans CJK SC", system-ui, sans-serif;
      background: #0b1018;
      color: #ecf3ff;
    }}
    * {{ box-sizing: border-box; }}
    body {{ margin: 0; min-height: 100vh; }}
    main {{ width: min(1500px, 96vw); margin: 0 auto; padding: 28px 0 36px; }}
    .title-row {{
      display: flex; align-items: end; justify-content: space-between;
      gap: 16px; margin-bottom: 20px;
    }}
    h1 {{ margin: 0; font-size: clamp(24px, 3vw, 38px); }}
    .subtitle {{ margin: 7px 0 0; color: #93a4bb; }}
    #server-state {{ color: #7dd3fc; white-space: nowrap; }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(min(100%, 520px), 1fr));
      gap: 18px;
    }}
    .camera-card {{
      overflow: hidden; border: 1px solid #25344a; border-radius: 16px;
      background: #111927; box-shadow: 0 14px 35px #0005;
    }}
    .camera-card header {{
      display: flex; align-items: center; justify-content: space-between;
      gap: 14px; padding: 15px 17px;
    }}
    h2 {{ margin: 0; font-size: 22px; }}
    .camera-card header p {{ margin: 4px 0 0; color: #93a4bb; font-size: 13px; }}
    .badge {{
      padding: 5px 9px; border-radius: 999px; background: #3f3015;
      color: #facc6b; font-size: 12px; font-weight: 700;
    }}
    .badge.ok {{ background: #143927; color: #6ee7a8; }}
    .badge.error {{ background: #491f29; color: #fda4af; }}
    .image-shell {{ aspect-ratio: 4 / 3; background: #03060a; }}
    img {{ width: 100%; height: 100%; object-fit: contain; display: block; }}
    dl {{
      display: grid; grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 1px; margin: 0; background: #25344a;
    }}
    dl div {{ padding: 11px 12px; background: #111927; min-width: 0; }}
    dt {{ color: #7f91a9; font-size: 11px; }}
    dd {{ margin: 4px 0 0; font-size: 13px; overflow-wrap: anywhere; }}
    .notice {{
      margin-top: 18px; padding: 13px 15px; border: 1px solid #25344a;
      border-radius: 12px; color: #b7c4d5; background: #101722;
    }}
    .dataset {{
      margin-top: 20px; border: 1px solid #25344a; border-radius: 16px;
      background: #111927; overflow: hidden; box-shadow: 0 14px 35px #0005;
    }}
    .dataset > header {{
      display: flex; justify-content: space-between; align-items: center;
      gap: 16px; padding: 17px 19px; border-bottom: 1px solid #25344a;
    }}
    .dataset > header p {{ margin: 5px 0 0; color: #93a4bb; }}
    .task {{
      margin: 16px 18px; padding: 13px 15px; border-radius: 11px;
      border: 1px solid #32435c; background: #0d1521;
    }}
    .task strong {{ color: #7dd3fc; margin-right: 8px; }}
    .data-layout {{
      display: grid; grid-template-columns: minmax(0, 1.65fr) minmax(280px, .75fr);
      gap: 0; border-top: 1px solid #25344a;
    }}
    .table-wrap {{ overflow-x: auto; }}
    table {{ width: 100%; border-collapse: collapse; min-width: 640px; }}
    th, td {{
      padding: 11px 14px; border-bottom: 1px solid #202d40;
      text-align: right; font-variant-numeric: tabular-nums;
    }}
    thead th {{ color: #91a5bf; background: #0d1521; font-size: 12px; }}
    tbody th {{ text-align: left; font-weight: 650; }}
    tbody th small {{
      display: block; color: #6f829b; font-size: 10px; font-weight: 400;
      margin-top: 3px;
    }}
    tbody td {{ font-family: "JetBrains Mono", ui-monospace, monospace; }}
    .health {{
      padding: 16px 17px; border-left: 1px solid #25344a;
      background: #0e1622;
    }}
    .health h3, .schema h3 {{ margin: 0 0 12px; font-size: 15px; }}
    .health-grid {{ display: grid; gap: 9px; }}
    .health-item {{
      display: flex; justify-content: space-between; gap: 12px;
      color: #91a5bf; font-size: 13px;
    }}
    .health-item output {{
      color: #e4edf9; text-align: right; overflow-wrap: anywhere;
      font-variant-numeric: tabular-nums;
    }}
    #piper-error {{
      display: none; margin-top: 13px; padding: 10px; border-radius: 9px;
      color: #fda4af; background: #3a1821; font-size: 12px;
      overflow-wrap: anywhere;
    }}
    .schema {{ padding: 16px 18px; border-top: 1px solid #25344a; }}
    .chips {{ display: flex; flex-wrap: wrap; gap: 8px; }}
    .chips span {{
      padding: 5px 8px; border: 1px solid #2d405a; border-radius: 8px;
      color: #a8bad0; background: #0c1420; font: 11px ui-monospace, monospace;
    }}
    .readonly {{ color: #6ee7a8; }}
    .readonly.bad {{ color: #fda4af; }}
    @media (max-width: 900px) {{
      .grid {{ grid-template-columns: 1fr; }}
      .title-row {{ align-items: start; flex-direction: column; }}
      .data-layout {{ grid-template-columns: 1fr; }}
      .health {{ border-left: 0; border-top: 1px solid #25344a; }}
    }}
  </style>
</head>
<body>
  <main>
    <div class="title-row">
      <div>
        <h1>Piper-VLA 训练数据实时检查</h1>
        <p class="subtitle">相机图像 + follower 状态 + master 动作 · 全程被动读取</p>
      </div>
      <div id="server-state">正在读取状态…</div>
    </div>
    <div class="grid">{''.join(cards)}</div>
    <section class="dataset">
      <header>
        <div>
          <h2>机械臂训练数据</h2>
          <p>observation.state 与 action 均为 7 维；J1–J6 单位为角度，夹爪单位为毫米。</p>
        </div>
        <span class="badge" id="piper-state">正在连接</span>
      </header>
      <div class="task"><strong>task</strong><span>{task_text}</span></div>
      <div class="data-layout">
        <div class="table-wrap">
          <table>
            <thead>
              <tr>
                <th>特征</th>
                <th>Follower observation</th>
                <th>Master action</th>
                <th>action − observation</th>
                <th>单位</th>
              </tr>
            </thead>
            <tbody>{''.join(feature_rows)}</tbody>
          </table>
        </div>
        <aside class="health">
          <h3>CAN 与采样健康状态</h3>
          <div class="health-grid">
            <div class="health-item"><span>CAN 接口</span><output id="can-interface">--</output></div>
            <div class="health-item"><span>采样帧率</span><output id="piper-fps">--</output></div>
            <div class="health-item"><span>样本数</span><output id="piper-samples">--</output></div>
            <div class="health-item"><span>数据年龄</span><output id="piper-age">--</output></div>
            <div class="health-item"><span>动作来源</span><output id="action-source">--</output></div>
            <div class="health-item"><span>接收 CAN 帧</span><output id="can-rx">--</output></div>
            <div class="health-item"><span>本程序新增 TX</span><output id="can-tx" class="readonly">--</output></div>
          </div>
          <div id="piper-error"></div>
        </aside>
      </div>
      <div class="schema">
        <h3>一个训练样本包含</h3>
        <div class="chips">
          {image_feature_chips}
          <span>observation.state[7]</span>
          <span>action[7]</span>
          <span>task</span>
          <span>timestamp</span>
          <span>frame_index</span>
          <span>episode_index</span>
        </div>
      </div>
    </section>
    <div class="notice">
      确认画面连续、关节数据持续刷新且“本程序新增 TX”为 0。结束时回到终端按
      <strong>Ctrl+C</strong>，程序会释放相机和被动 CAN 读取器。
    </div>
  </main>
  <script>
    const cameraDomIds = {camera_dom_json};
    const previous = {{}};

    function number(value, digits = 1) {{
      return Number.isFinite(value) ? value.toFixed(digits) : "--";
    }}

    function updatePiper(piper) {{
      const badge = document.getElementById("piper-state");
      const errorBox = document.getElementById("piper-error");
      const hasSample = Boolean(piper.observation && piper.action);
      const healthy = piper.connected && hasSample && !piper.error;
      badge.textContent = healthy
        ? "数据正常"
        : (piper.starting ? "正在连接" : (piper.enabled ? "不可用" : "仅相机"));
      badge.className = `badge ${{healthy ? "ok" : "error"}}`;

      for (const row of document.querySelectorAll("tr[data-feature]")) {{
        const feature = row.dataset.feature;
        row.querySelector('[data-value="observation"]').textContent =
          hasSample ? number(piper.observation[feature], 3) : "--";
        row.querySelector('[data-value="action"]').textContent =
          hasSample ? number(piper.action[feature], 3) : "--";
        row.querySelector('[data-value="delta"]').textContent =
          hasSample ? number(piper.delta[feature], 3) : "--";
      }}

      const can = piper.can || {{}};
      const health = piper.health || {{}};
      document.getElementById("can-interface").textContent =
        can.interface ? `${{can.interface}} · ${{can.bitrate / 1000000}} Mbps` : "--";
      document.getElementById("piper-fps").textContent =
        piper.sample_fps == null ? "--" : `${{number(piper.sample_fps)}} FPS`;
      document.getElementById("piper-samples").textContent =
        piper.sample_count ?? "--";
      document.getElementById("piper-age").textContent =
        piper.latest_sample_age_ms == null
          ? "--"
          : `${{number(piper.latest_sample_age_ms)}} ms`;
      document.getElementById("action-source").textContent =
        health.action_source === "master_target"
          ? "master 目标"
          : (health.action_source === "follower_feedback" ? "follower 保持值" : "--");
      document.getElementById("can-rx").textContent =
        health.frames_received ?? "--";

      const tx = can.tx_packets_delta;
      const txOutput = document.getElementById("can-tx");
      txOutput.textContent = tx == null ? "--" : String(tx);
      txOutput.className = `readonly ${{tx === 0 ? "" : "bad"}}`;

      if (piper.error) {{
        errorBox.textContent = piper.error;
        errorBox.style.display = "block";
      }} else {{
        errorBox.textContent = "";
        errorBox.style.display = "none";
      }}
    }}

    async function updateStatus() {{
      const now = performance.now();
      try {{
        const response = await fetch("/status.json", {{cache: "no-store"}});
        const payload = await response.json();
        document.getElementById("server-state").textContent =
          `运行 ${{number(payload.uptime_s, 0)}} 秒`;
        updatePiper(payload.piper);

        for (const [name, data] of Object.entries(payload.cameras)) {{
          const card = document.getElementById(cameraDomIds[name]);
          const capture = data.capture;
          const viewer = data.viewer;
          const badge = card.querySelector('[data-field="state"]');
          const healthy = capture.connected && capture.background_thread_alive && !viewer.error;
          badge.textContent = healthy ? "正常" : (viewer.error || "异常");
          badge.className = `badge ${{healthy ? "ok" : "error"}}`;

          const old = previous[name];
          let fps = null;
          if (old && capture.frames_received >= old.frames) {{
            fps = (capture.frames_received - old.frames) * 1000 / (now - old.time);
          }}
          previous[name] = {{frames: capture.frames_received, time: now}};

          card.querySelector('[data-field="fps"]').textContent =
            fps === null ? "计算中" : `${{number(fps)}} FPS`;
          card.querySelector('[data-field="frames"]').textContent =
            capture.frames_received ?? "--";
          card.querySelector('[data-field="age"]').textContent =
            `${{number(capture.latest_frame_age_ms)}} ms`;

          const errorCount =
            (capture.frame_index_gaps || 0) +
            (capture.duplicate_frame_indices || 0) +
            (capture.hardware_timestamp_regressions || 0) +
            (capture.timeouts || 0) +
            (capture.read_failures || 0) +
            (capture.bad_color_frames || 0);
          card.querySelector('[data-field="errors"]').textContent =
            errorCount === 0 ? "0" : `${{errorCount}}（请停止检查）`;
        }}
      }} catch (error) {{
        document.getElementById("server-state").textContent = `状态读取失败：${{error}}`;
      }}
    }}

    updateStatus();
    setInterval(updateStatus, 1000);
  </script>
</body>
</html>
"""
    return page.encode("utf-8")


class ViewerRequestHandler(BaseHTTPRequestHandler):
    viewer_state: ViewerState
    index_html: bytes

    def do_GET(self) -> None:
        path = self.path.split("?", maxsplit=1)[0]
        if path == "/":
            self._send_bytes("text/html; charset=utf-8", self.index_html)
            return
        if path == "/status.json":
            payload = json.dumps(
                self.viewer_state.status(),
                ensure_ascii=False,
                allow_nan=False,
            ).encode("utf-8")
            self._send_bytes("application/json; charset=utf-8", payload, no_cache=True)
            return
        if path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return
        if path.startswith("/stream/") and path.endswith(".mjpg"):
            name = path[len("/stream/") : -len(".mjpg")]
            if name in self.viewer_state.buffers:
                self._stream_camera(name)
                return
        self.send_error(404, "Not found")

    def _send_bytes(self, content_type: str, payload: bytes, *, no_cache: bool = False) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        if no_cache:
            self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _stream_camera(self, name: str) -> None:
        stream = self.viewer_state.buffers[name]
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

        last_sequence = -1
        try:
            while not self.viewer_state.stop_event.is_set():
                with stream.condition:
                    stream.condition.wait_for(
                        lambda: (
                            stream.sequence != last_sequence
                            or self.viewer_state.stop_event.is_set()
                        ),
                        timeout=2.0,
                    )
                    if self.viewer_state.stop_event.is_set():
                        return
                    if stream.jpeg is None or stream.sequence == last_sequence:
                        continue
                    jpeg = stream.jpeg
                    last_sequence = stream.sequence

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return

    def log_message(self, format_string: str, *args: Any) -> None:
        if not self.path.startswith("/status.json"):
            sys.stderr.write(
                f"{self.address_string()} - {format_string % args}\n"
            )


class ViewerHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def load_camera_configs(path: Path) -> tuple[dict[str, Any], dict[str, OrbbecCameraConfig]]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    capture = raw["capture"]
    configs: dict[str, OrbbecCameraConfig] = {}
    serials: list[str] = []
    for name, camera in raw["cameras"].items():
        serial = str(camera["serial_number"])
        serials.append(serial)
        configs[name] = OrbbecCameraConfig(
            serial_number=serial,
            width=int(capture["width"]),
            height=int(capture["height"]),
            fps=int(capture["fps"]),
            color_mode=capture["color_mode"],
            use_depth=bool(capture["depth_enabled"]),
            timeout_ms=int(capture["timeout_ms"]),
            warmup_s=float(capture["warmup_s"]),
        )

    if not configs:
        raise ValueError("At least one camera must be present in the configuration.")
    if len(set(serials)) != len(serials):
        raise ValueError("Configured camera serial numbers must be unique.")
    return raw, configs


def validate_args(args: argparse.Namespace) -> None:
    if not 1 <= args.port <= 65535:
        raise ValueError("--port must be between 1 and 65535.")
    if args.display_fps <= 0:
        raise ValueError("--display-fps must be positive.")
    if not 1 <= args.jpeg_quality <= 100:
        raise ValueError("--jpeg-quality must be between 1 and 100.")
    if args.max_frame_age_ms <= 0:
        raise ValueError("--max-frame-age-ms must be positive.")
    if args.piper_refresh_fps <= 0:
        raise ValueError("--piper-refresh-fps must be positive.")
    if not args.task.strip():
        raise ValueError("--task must not be empty.")
    if args.camera_only and args.require_piper:
        raise ValueError("--camera-only and --require-piper cannot be combined.")


def select_connected_configs(
    camera_configs: dict[str, OrbbecCameraConfig],
    found_serials: set[str],
    requested_names: set[str],
) -> tuple[dict[str, OrbbecCameraConfig], list[str]]:
    selected = {
        name: camera_config
        for name, camera_config in camera_configs.items()
        if name in requested_names and camera_config.serial_number in found_serials
    }
    missing_names = [
        name
        for name, camera_config in camera_configs.items()
        if name in requested_names and camera_config.serial_number not in found_serials
    ]
    return selected, missing_names


def main() -> int:
    args = parse_args()
    validate_args(args)
    raw_config, camera_configs = load_camera_configs(args.config)

    requested_names = set(args.camera or camera_configs)
    unknown_names = sorted(requested_names - set(camera_configs))
    if unknown_names:
        raise ValueError(
            f"Unknown logical camera names: {unknown_names}. "
            f"Configured names: {sorted(camera_configs)}"
        )

    found = OrbbecCamera.find_cameras()
    found_serials = {str(device["serial_number"]) for device in found}
    selected_configs, missing_names = select_connected_configs(
        camera_configs,
        found_serials,
        requested_names,
    )
    if missing_names:
        missing_text = ", ".join(
            f"{name}={camera_configs[name].serial_number}" for name in missing_names
        )
        print(f"提示：以下配置相机当前未连接，已跳过：{missing_text}", file=sys.stderr)

    if not selected_configs:
        raise ConnectionError(
            "None of the selected configured cameras are connected. "
            f"Connected Orbbec serial numbers: {sorted(found_serials)}"
        )

    cameras = {
        name: OrbbecCamera(camera_config)
        for name, camera_config in selected_configs.items()
    }
    connected: list[OrbbecCamera] = []
    piper_source = (
        None
        if args.camera_only
        else PiperDataSource(
            args.piper_config,
            refresh_fps=args.piper_refresh_fps,
        )
    )
    state: ViewerState | None = None
    server: ViewerHTTPServer | None = None

    try:
        with ThreadPoolExecutor(max_workers=len(cameras)) as executor:
            futures = {
                name: executor.submit(camera.connect)
                for name, camera in cameras.items()
            }
            for name, future in futures.items():
                future.result()
                connected.append(cameras[name])

        state = ViewerState(
            raw_config,
            cameras,
            display_fps=args.display_fps,
            jpeg_quality=args.jpeg_quality,
            max_frame_age_ms=args.max_frame_age_ms,
            task=args.task,
            piper=piper_source,
        )
        state.start_encoders()
        state.wait_until_ready()
        if piper_source is not None:
            try:
                piper_source.start()
            except Exception as exc:
                if args.require_piper:
                    raise
                print(
                    "提示：机械臂被动数据当前不可用，网页仍将显示相机："
                    f"{type(exc).__name__}: {exc}",
                    file=sys.stderr,
                )

        ViewerRequestHandler.viewer_state = state
        ViewerRequestHandler.index_html = build_index_html(state)
        server = ViewerHTTPServer((args.host, args.port), ViewerRequestHandler)

        browser_host = "127.0.0.1" if args.host in {"0.0.0.0", "::"} else args.host
        url = f"http://{browser_host}:{server.server_port}/"
        print(f"训练数据实时网页已启动（{len(cameras)} 路相机）：")
        for name, camera in cameras.items():
            print(f"  {name}: {camera.serial_number} ({camera.width}x{camera.height}@{camera.fps})")
        print(f"浏览器地址：{url}")
        if args.camera_only:
            print("机械臂数据：已禁用（--camera-only）")
        elif piper_source is not None and piper_source.status()["connected"]:
            print(
                f"机械臂数据：{piper_source.interface} 被动接收，"
                "不发送 CAN 报文"
            )
        else:
            print("机械臂数据：当前不可用，原因显示在网页中")
        print("结束时按 Ctrl+C。")

        if not args.no_browser:
            Thread(target=webbrowser.open, args=(url,), daemon=True).start()

        try:
            server.serve_forever(poll_interval=0.2)
        except KeyboardInterrupt:
            print("\n正在关闭相机实时画面……")
    finally:
        if server is not None:
            server.server_close()
        if state is not None:
            state.stop()
        elif piper_source is not None:
            piper_source.stop()
        for camera in reversed(connected):
            if camera.is_connected:
                camera.disconnect()
        print("相机和被动数据读取器已释放。")

    return 0


if __name__ == "__main__":
    sys.exit(main())
