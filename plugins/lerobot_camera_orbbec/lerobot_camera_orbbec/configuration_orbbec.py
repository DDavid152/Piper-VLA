from dataclasses import dataclass

from lerobot.cameras import CameraConfig, ColorMode, Cv2Rotation


@CameraConfig.register_subclass("orbbec")
@dataclass
class OrbbecCameraConfig(CameraConfig):
    """Configuration for an Orbbec SDK v2 color camera.

    Devices are selected exclusively by serial number. The first Piper-VLA
    dataset stage uses RGB only; depth support is available for diagnostics but
    remains disabled by default.
    """

    serial_number: str
    color_mode: ColorMode = ColorMode.RGB
    use_depth: bool = False
    rotation: Cv2Rotation = Cv2Rotation.NO_ROTATION
    timeout_ms: int = 1000
    warmup_s: float = 2.0
    max_consecutive_failures: int = 10

    def __post_init__(self) -> None:
        self.color_mode = ColorMode(self.color_mode)
        self.rotation = Cv2Rotation(self.rotation)

        if not self.serial_number.strip():
            raise ValueError("`serial_number` must not be empty.")

        values = (self.fps, self.width, self.height)
        if any(value is None for value in values):
            raise ValueError("`fps`, `width` and `height` must all be configured.")

        if self.fps is not None and self.fps <= 0:
            raise ValueError("`fps` must be positive.")
        if self.width is not None and self.width <= 0:
            raise ValueError("`width` must be positive.")
        if self.height is not None and self.height <= 0:
            raise ValueError("`height` must be positive.")
        if self.timeout_ms <= 0:
            raise ValueError("`timeout_ms` must be positive.")
        if self.warmup_s < 0:
            raise ValueError("`warmup_s` must be non-negative.")
        if self.max_consecutive_failures <= 0:
            raise ValueError("`max_consecutive_failures` must be positive.")
