# LeRobot Orbbec camera plugin

This package adds the `orbbec` camera type to LeRobot v0.6.0. It targets
Orbbec SDK v2 cameras and selects devices by immutable serial number rather
than `/dev/video*` enumeration order.

The Piper-VLA workstation configuration is stored in
`../../config/cameras.json`:

- `front`: Gemini 335L `CP28563000XR`, remote fixed view;
- `wrist`: Gemini 335L `CP28563000XP`, end-effector view;
- RGB: 640×480 at 30 FPS;
- depth: disabled for the first SmolVLA dataset stage.

Install the plugin in the existing environment:

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python -m pip install --editable /home/ubuntu22/Piper-VLA/plugins/lerobot_camera_orbbec
```

Minimal use:

```python
from lerobot_camera_orbbec import OrbbecCamera, OrbbecCameraConfig

camera = OrbbecCamera(
    OrbbecCameraConfig(
        serial_number="CP28563000XR",
        width=640,
        height=480,
        fps=30,
    )
)
camera.connect()
image = camera.async_read()
camera.disconnect()
```

`OrbbecCamera` captures continuously in a background thread. A caller receives
the newest unconsumed frame through `async_read()` or can inspect the current
buffer through `read_latest()`. Timeouts, frame-index gaps, duplicate indices,
timestamp regressions and the latest frame age are available from
`get_health_stats()`.

The optional depth API exists for hardware diagnostics, but project recording
must leave `use_depth=False` until the dataset schema is deliberately revised.
