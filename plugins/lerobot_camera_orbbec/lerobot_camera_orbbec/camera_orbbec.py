import time
from threading import Condition, Event, Thread
from typing import Any

import numpy as np
from lerobot.cameras import Camera, ColorMode, Cv2Rotation
from lerobot.utils.errors import DeviceNotConnectedError
from numpy.typing import NDArray

from .configuration_orbbec import OrbbecCameraConfig


class OrbbecCamera(Camera):
    """LeRobot camera backed by Orbbec SDK v2.

    The SDK pipeline is consumed continuously in a background thread. Frames are
    detached from SDK-owned buffers before publication, and failures are surfaced
    to callers rather than replaced with a synthetic or stale black image.
    """

    config_class = OrbbecCameraConfig
    name = "orbbec"

    def __init__(self, config: OrbbecCameraConfig):
        super().__init__(config)
        self.config = config
        self.serial_number = config.serial_number
        self.color_mode = config.color_mode
        self.use_depth = config.use_depth
        self.rotation = config.rotation
        self.timeout_ms = config.timeout_ms
        self.warmup_s = config.warmup_s
        self.max_consecutive_failures = config.max_consecutive_failures

        if self.width is None or self.height is None or self.fps is None:
            raise ValueError("Orbbec camera resolution and FPS must be configured.")

        self.capture_width = self.width
        self.capture_height = self.height
        if self.rotation in (Cv2Rotation.ROTATE_90, Cv2Rotation.ROTATE_270):
            self.capture_width, self.capture_height = self.height, self.width

        self._ob: Any | None = None
        self._context: Any | None = None
        self._pipeline: Any | None = None
        self._connected = False
        self._stop_event: Event | None = None
        self._thread: Thread | None = None
        self._condition = Condition()
        self._thread_error: BaseException | None = None

        self._latest_color: NDArray[np.uint8] | None = None
        self._latest_depth: NDArray[np.uint16] | None = None
        self._latest_capture_time: float | None = None
        self._latest_metadata: dict[str, int | float] | None = None
        self._last_read_metadata: dict[str, int | float] | None = None
        self._generation = 0
        self._last_color_generation = 0
        self._last_depth_generation = 0

        self._stats: dict[str, int | float | None] = {}
        self._reset_stats()

    def __str__(self) -> str:
        return f"{self.__class__.__name__}({self.serial_number})"

    @property
    def is_connected(self) -> bool:
        return self._connected and self._pipeline is not None

    @staticmethod
    def _load_sdk() -> Any:
        try:
            import pyorbbecsdk as ob
        except ImportError as exc:
            raise ImportError(
                "pyorbbecsdk2 is required for the Orbbec camera plugin. "
                "Install the Piper-VLA pinned environment first."
            ) from exc
        return ob

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        ob = OrbbecCamera._load_sdk()
        context = ob.Context()
        devices = context.query_devices()
        found: list[dict[str, Any]] = []
        for index in range(devices.get_count()):
            device = devices.get_device_by_index(index)
            info = device.get_device_info()
            found.append(
                {
                    "name": info.get_name(),
                    "type": "Orbbec",
                    "id": info.get_serial_number(),
                    "serial_number": info.get_serial_number(),
                    "firmware_version": info.get_firmware_version(),
                    "hardware_version": info.get_hardware_version(),
                    "connection_type": info.get_connection_type(),
                    "uid": info.get_uid(),
                    "vid": f"0x{info.get_vid():04x}",
                    "pid": f"0x{info.get_pid():04x}",
                }
            )
        return found

    def _reset_stats(self) -> None:
        self._stats = {
            "frames_received": 0,
            "timeouts": 0,
            "read_failures": 0,
            "missing_color_frames": 0,
            "missing_depth_frames": 0,
            "bad_color_frames": 0,
            "bad_depth_frames": 0,
            "frame_index_gaps": 0,
            "duplicate_frame_indices": 0,
            "hardware_timestamp_regressions": 0,
            "last_frame_index": None,
            "last_hardware_timestamp_us": None,
            "last_system_timestamp_us": None,
        }

    def _select_video_profile(self, pipeline: Any, sensor: Any, pixel_format: Any) -> Any:
        profiles = pipeline.get_stream_profile_list(sensor)
        try:
            return profiles.get_video_stream_profile(
                self.capture_width,
                self.capture_height,
                pixel_format,
                self.fps,
            )
        except Exception as exc:
            available: list[str] = []
            for index in range(profiles.get_count()):
                profile = profiles.get_stream_profile_by_index(index).as_video_stream_profile()
                if profile.get_width() == self.capture_width and profile.get_height() == self.capture_height:
                    available.append(
                        f"{profile.get_width()}x{profile.get_height()}@{profile.get_fps()} "
                        f"{profile.get_format().name}"
                    )
            raise ValueError(
                f"{self} does not support the requested "
                f"{self.capture_width}x{self.capture_height}@{self.fps} {pixel_format.name}. "
                f"Matching-resolution profiles: {sorted(set(available))}"
            ) from exc

    def connect(self, warmup: bool = True) -> None:
        if self.is_connected:
            raise RuntimeError(f"{self} is already connected.")

        ob = self._load_sdk()
        context = ob.Context()
        devices = context.query_devices()
        serials = [
            devices.get_device_serial_number_by_index(index) for index in range(devices.get_count())
        ]
        if self.serial_number not in serials:
            raise ConnectionError(
                f"{self} was not found. Connected Orbbec serial numbers: {serials}"
            )

        device = devices.get_device_by_serial_number(self.serial_number)
        pipeline = ob.Pipeline(device)
        sdk_config = ob.Config()
        color_profile = self._select_video_profile(
            pipeline,
            ob.OBSensorType.COLOR_SENSOR,
            ob.OBFormat.RGB,
        )
        sdk_config.enable_stream(color_profile)
        if self.use_depth:
            depth_profile = self._select_video_profile(
                pipeline,
                ob.OBSensorType.DEPTH_SENSOR,
                ob.OBFormat.Y16,
            )
            sdk_config.enable_stream(depth_profile)

        self._reset_stats()
        self._thread_error = None
        self._latest_color = None
        self._latest_depth = None
        self._latest_capture_time = None
        self._latest_metadata = None
        self._last_read_metadata = None
        self._generation = 0
        self._last_color_generation = 0
        self._last_depth_generation = 0

        try:
            pipeline.start(sdk_config)
            self._ob = ob
            self._context = context
            self._pipeline = pipeline
            self._connected = True
            self._start_read_thread()
            if warmup:
                self._wait_for_warmup()
        except Exception:
            self._disconnect_internal()
            raise

    def _wait_for_warmup(self) -> None:
        start = time.perf_counter()
        deadline = start + max(self.warmup_s, self.timeout_ms / 1000.0) + 2.0
        with self._condition:
            while True:
                if self._thread_error is not None:
                    raise ConnectionError(f"{self} failed during warmup.") from self._thread_error
                warmed = (
                    self._latest_color is not None
                    and (not self.use_depth or self._latest_depth is not None)
                    and time.perf_counter() - start >= self.warmup_s
                )
                if warmed:
                    return
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    raise ConnectionError(
                        f"{self} did not produce valid frames during {self.warmup_s:.1f}s warmup."
                    )
                self._condition.wait(timeout=min(remaining, 0.1))

    def _start_read_thread(self) -> None:
        self._stop_event = Event()
        self._thread = Thread(
            target=self._read_loop,
            name=f"{self}_read_loop",
            daemon=True,
        )
        self._thread.start()

    def _rotate(self, image: NDArray[Any]) -> NDArray[Any]:
        if self.rotation == Cv2Rotation.ROTATE_90:
            return np.rot90(image, k=3).copy()
        if self.rotation == Cv2Rotation.ROTATE_180:
            return np.rot90(image, k=2).copy()
        if self.rotation == Cv2Rotation.ROTATE_270:
            return np.rot90(image, k=1).copy()
        return image

    def _decode_color(self, frame: Any) -> NDArray[np.uint8]:
        data = np.frombuffer(frame.get_data(), dtype=np.uint8)
        expected = self.capture_width * self.capture_height * 3
        if data.size != expected:
            raise RuntimeError(f"{self} color frame has {data.size} bytes; expected {expected}.")
        image = data.reshape(self.capture_height, self.capture_width, 3).copy()
        if self.color_mode == ColorMode.BGR:
            image = image[..., ::-1].copy()
        image = self._rotate(image)
        if image.shape != (self.height, self.width, 3):
            raise RuntimeError(
                f"{self} produced color shape {image.shape}; expected "
                f"{(self.height, self.width, 3)}."
            )
        return image

    def _decode_depth(self, frame: Any) -> NDArray[np.uint16]:
        data = np.frombuffer(frame.get_data(), dtype=np.uint16)
        expected = self.capture_width * self.capture_height
        if data.size != expected:
            raise RuntimeError(f"{self} depth frame has {data.size} pixels; expected {expected}.")
        depth = data.reshape(self.capture_height, self.capture_width).copy()
        depth = self._rotate(depth)
        if depth.shape != (self.height, self.width):
            raise RuntimeError(
                f"{self} produced depth shape {depth.shape}; expected {(self.height, self.width)}."
            )
        return depth[..., np.newaxis]

    def _update_frame_stats(self, frame: Any) -> dict[str, int]:
        frame_index = int(frame.get_index())
        hardware_timestamp_us = int(frame.get_timestamp_us())
        system_timestamp_us = int(frame.get_system_timestamp_us())

        last_index = self._stats["last_frame_index"]
        if isinstance(last_index, int):
            if frame_index == last_index:
                self._stats["duplicate_frame_indices"] = (
                    int(self._stats["duplicate_frame_indices"]) + 1
                )
            elif frame_index > last_index + 1:
                self._stats["frame_index_gaps"] = (
                    int(self._stats["frame_index_gaps"]) + frame_index - last_index - 1
                )

        last_timestamp = self._stats["last_hardware_timestamp_us"]
        if isinstance(last_timestamp, int) and hardware_timestamp_us <= last_timestamp:
            self._stats["hardware_timestamp_regressions"] = (
                int(self._stats["hardware_timestamp_regressions"]) + 1
            )

        self._stats["last_frame_index"] = frame_index
        self._stats["last_hardware_timestamp_us"] = hardware_timestamp_us
        self._stats["last_system_timestamp_us"] = system_timestamp_us
        self._stats["frames_received"] = int(self._stats["frames_received"]) + 1
        return {
            "frame_index": frame_index,
            "hardware_timestamp_us": hardware_timestamp_us,
            "system_timestamp_us": system_timestamp_us,
        }

    def _read_loop(self) -> None:
        if self._pipeline is None or self._stop_event is None:
            return

        consecutive_failures = 0
        while not self._stop_event.is_set():
            try:
                frames = self._pipeline.wait_for_frames(self.timeout_ms)
                if frames is None:
                    self._stats["timeouts"] = int(self._stats["timeouts"]) + 1
                    raise TimeoutError(f"{self} timed out waiting for an SDK frame set.")

                color_frame = frames.get_color_frame()
                if color_frame is None:
                    self._stats["missing_color_frames"] = (
                        int(self._stats["missing_color_frames"]) + 1
                    )
                    raise RuntimeError(f"{self} frame set does not contain a color frame.")
                try:
                    color_image = self._decode_color(color_frame)
                except Exception:
                    self._stats["bad_color_frames"] = int(self._stats["bad_color_frames"]) + 1
                    raise

                depth_image: NDArray[np.uint16] | None = None
                if self.use_depth:
                    depth_frame = frames.get_depth_frame()
                    if depth_frame is None:
                        self._stats["missing_depth_frames"] = (
                            int(self._stats["missing_depth_frames"]) + 1
                        )
                        raise RuntimeError(f"{self} frame set does not contain a depth frame.")
                    try:
                        depth_image = self._decode_depth(depth_frame)
                    except Exception:
                        self._stats["bad_depth_frames"] = (
                            int(self._stats["bad_depth_frames"]) + 1
                        )
                        raise

                capture_time = time.perf_counter()
                with self._condition:
                    metadata = self._update_frame_stats(color_frame)
                    self._latest_color = color_image
                    self._latest_depth = depth_image
                    self._latest_capture_time = capture_time
                    self._latest_metadata = {
                        **metadata,
                        "capture_monotonic_s": capture_time,
                    }
                    self._generation += 1
                    consecutive_failures = 0
                    self._condition.notify_all()
            except Exception as exc:
                if self._stop_event.is_set():
                    break
                consecutive_failures += 1
                self._stats["read_failures"] = int(self._stats["read_failures"]) + 1
                if consecutive_failures >= self.max_consecutive_failures:
                    with self._condition:
                        self._thread_error = exc
                        self._condition.notify_all()
                    return

    def _raise_if_unavailable(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if self._thread is None or not self._thread.is_alive():
            if self._thread_error is not None:
                raise RuntimeError(f"{self} background capture failed.") from self._thread_error
            raise RuntimeError(f"{self} background capture thread is not running.")

    def _wait_for_frame(self, *, depth: bool, timeout_ms: float) -> NDArray[Any]:
        self._raise_if_unavailable()
        deadline = time.perf_counter() + timeout_ms / 1000.0
        with self._condition:
            consumed_generation = (
                self._last_depth_generation if depth else self._last_color_generation
            )
            while self._generation <= consumed_generation:
                if self._thread_error is not None:
                    raise RuntimeError(f"{self} background capture failed.") from self._thread_error
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    raise TimeoutError(
                        f"Timed out waiting {timeout_ms:.0f} ms for a fresh frame from {self}."
                    )
                self._condition.wait(timeout=remaining)

            frame = self._latest_depth if depth else self._latest_color
            if frame is None:
                raise RuntimeError(f"{self} published a frame generation without image data.")
            if depth:
                self._last_depth_generation = self._generation
            else:
                self._last_color_generation = self._generation
            if self._latest_metadata is None:
                raise RuntimeError(f"{self} published a frame without metadata.")
            self._last_read_metadata = dict(self._latest_metadata)
            return frame

    def read(self) -> NDArray[np.uint8]:
        return self._wait_for_frame(depth=False, timeout_ms=max(self.timeout_ms, 10000))

    def async_read(self, timeout_ms: float = 200) -> NDArray[np.uint8]:
        return self._wait_for_frame(depth=False, timeout_ms=timeout_ms)

    def read_depth(self) -> NDArray[np.uint16]:
        if not self.use_depth:
            raise RuntimeError(f"{self} depth stream is disabled.")
        return self._wait_for_frame(depth=True, timeout_ms=max(self.timeout_ms, 10000))

    def async_read_depth(self, timeout_ms: float = 200) -> NDArray[np.uint16]:
        if not self.use_depth:
            raise RuntimeError(f"{self} depth stream is disabled.")
        return self._wait_for_frame(depth=True, timeout_ms=timeout_ms)

    def _read_latest(self, *, depth: bool, max_age_ms: int) -> NDArray[Any]:
        self._raise_if_unavailable()
        with self._condition:
            frame = self._latest_depth if depth else self._latest_color
            capture_time = self._latest_capture_time
            if frame is None or capture_time is None:
                raise RuntimeError(f"{self} has not captured a frame yet.")
            age_ms = (time.perf_counter() - capture_time) * 1000
            if age_ms > max_age_ms:
                raise TimeoutError(
                    f"{self} latest frame is {age_ms:.1f} ms old; limit is {max_age_ms} ms."
                )
            return frame

    def read_latest(self, max_age_ms: int = 500) -> NDArray[np.uint8]:
        return self._read_latest(depth=False, max_age_ms=max_age_ms)

    def read_latest_depth(self, max_age_ms: int = 500) -> NDArray[np.uint16]:
        if not self.use_depth:
            raise RuntimeError(f"{self} depth stream is disabled.")
        return self._read_latest(depth=True, max_age_ms=max_age_ms)

    def get_last_frame_metadata(self) -> dict[str, int | float]:
        self._raise_if_unavailable()
        with self._condition:
            if self._last_read_metadata is None:
                raise RuntimeError(f"{self} has not returned a frame with metadata yet.")
            return dict(self._last_read_metadata)

    def get_health_stats(self) -> dict[str, int | float | None]:
        with self._condition:
            stats = dict(self._stats)
            if self._latest_capture_time is None:
                stats["latest_frame_age_ms"] = None
            else:
                stats["latest_frame_age_ms"] = (
                    time.perf_counter() - self._latest_capture_time
                ) * 1000
            stats["background_thread_alive"] = bool(
                self._thread is not None and self._thread.is_alive()
            )
            stats["connected"] = self.is_connected
            return stats

    def _disconnect_internal(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=self.timeout_ms / 1000.0 + 1.0)

        pipeline = self._pipeline
        self._pipeline = None
        self._connected = False
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass

        self._thread = None
        self._stop_event = None
        self._context = None
        self._ob = None
        with self._condition:
            self._latest_color = None
            self._latest_depth = None
            self._latest_capture_time = None
            self._latest_metadata = None
            self._last_read_metadata = None
            self._condition.notify_all()

    def disconnect(self) -> None:
        if not self.is_connected and self._pipeline is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self._disconnect_internal()
