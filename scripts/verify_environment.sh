#!/usr/bin/env bash
set -euo pipefail

readonly PROJECT_ROOT="/home/ubuntu22/Piper-VLA"
readonly CONDA_ROOT="/home/ubuntu22/miniforge3"
readonly ENV_NAME="lerobot"
readonly LOG_DIR="${PROJECT_ROOT}/logs"
readonly RUN_ID="$(date +%Y%m%d_%H%M%S)"
readonly LOG_FILE="${LOG_DIR}/environment_verify_${RUN_ID}.log"

mkdir -p "${LOG_DIR}"
exec > >(tee "${LOG_FILE}") 2>&1

source "${CONDA_ROOT}/etc/profile.d/conda.sh"
conda activate "${ENV_NAME}"

echo "Verification log: ${LOG_FILE}"
echo "Python: $(python --version 2>&1)"
echo "Executable: $(command -v python)"

python -m pip check

python - <<'PY'
import importlib
import json

modules = [
    "torch",
    "torchvision",
    "lerobot",
    "av",
    "torchcodec",
    "can",
    "piper_sdk",
    "pyorbbecsdk",
]

versions = {}
for name in modules:
    module = importlib.import_module(name)
    versions[name] = getattr(module, "__version__", "imported")

print(json.dumps(versions, indent=2, ensure_ascii=False))
PY

python - <<'PY'
import torch

assert torch.cuda.is_available(), "CUDA is not available"
assert torch.version.cuda == "12.8", torch.version.cuda

device_name = torch.cuda.get_device_name(0)
a = torch.randn((512, 512), device="cuda")
b = torch.randn((512, 512), device="cuda")
c = a @ b
torch.cuda.synchronize()

assert c.is_cuda
assert torch.isfinite(c).all()
print(f"CUDA smoke test passed: {device_name}, CUDA {torch.version.cuda}")
PY

lerobot-record --help >/dev/null
lerobot-train --help >/dev/null
lerobot-dataset-viz --help >/dev/null
echo "LeRobot CLI smoke tests passed."

tmp_dir="$(mktemp -d)"
trap 'rm -rf -- "${tmp_dir}"' EXIT
export PIPER_VLA_VIDEO_TEST="${tmp_dir}/synthetic.mp4"

python - <<'PY'
import os

import av
import numpy as np

path = os.environ["PIPER_VLA_VIDEO_TEST"]
container = av.open(path, mode="w")
stream = container.add_stream("libx264", rate=30)
stream.width = 640
stream.height = 480
stream.pix_fmt = "yuv420p"

for index in range(30):
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:, :, 0] = index * 8
    image[:, :, 1] = np.arange(640, dtype=np.uint8)[None, :]
    frame = av.VideoFrame.from_ndarray(image, format="rgb24")
    for packet in stream.encode(frame):
        container.mux(packet)

for packet in stream.encode():
    container.mux(packet)
container.close()

decoded = 0
with av.open(path) as input_container:
    for _ in input_container.decode(video=0):
        decoded += 1

assert decoded == 30, decoded
print(f"PyAV encode/decode passed: {decoded} frames")
PY

python - <<'PY'
import os
from torchcodec.decoders import VideoDecoder

decoder = VideoDecoder(os.environ["PIPER_VLA_VIDEO_TEST"])
assert len(decoder) == 30, len(decoder)
frame = decoder[0]
assert tuple(frame.shape) == (3, 480, 640), tuple(frame.shape)
print(f"TorchCodec decode passed: {len(decoder)} frames")
PY

orbbec_library="$(
  python - <<'PY'
import importlib.util

spec = importlib.util.find_spec("pyorbbecsdk")
if spec is None or spec.origin is None:
    raise SystemExit("pyorbbecsdk native module not found")
print(spec.origin)
PY
)"

echo "Orbbec module: ${orbbec_library}"
ldd "${orbbec_library}"
if ldd "${orbbec_library}" | grep -q "not found"; then
  echo "Orbbec native module has missing shared libraries." >&2
  exit 1
fi
echo "Orbbec native dependency check passed."

python - <<'PY'
import json

from pyorbbecsdk import Context

context = Context()
device_list = context.query_devices()
devices = []
for index in range(device_list.get_count()):
    device = device_list.get_device_by_index(index)
    info = device.get_device_info()
    devices.append(
        {
            "name": info.get_name(),
            "serial_number": info.get_serial_number(),
            "firmware_version": info.get_firmware_version(),
        }
    )

print(
    "Read-only Orbbec enumeration:\n"
    + json.dumps(
        {"device_count": len(devices), "devices": devices},
        indent=2,
        ensure_ascii=False,
    )
)
PY

echo "All environment checks passed."
