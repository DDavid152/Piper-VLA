#!/usr/bin/env bash
set -euo pipefail

readonly PROJECT_ROOT="/home/ubuntu22/Piper-VLA"
readonly CONDA_ROOT="/home/ubuntu22/miniforge3"
readonly ENV_NAME="lerobot"
readonly LEROBOT_DIR="${PROJECT_ROOT}/third_party/lerobot"
readonly LEROBOT_COMMIT="30da8e687a6dfc617fcd94afc367ac7071c376ce"
readonly PYTORCH_INDEX="https://download.pytorch.org/whl/cu128"

if [[ ! -x "${CONDA_ROOT}/bin/conda" ]]; then
  echo "Miniforge is missing at ${CONDA_ROOT}." >&2
  exit 1
fi

if ! "${CONDA_ROOT}/bin/conda" env list \
  | awk '{print $1}' \
  | grep -qx "${ENV_NAME}"; then
  "${CONDA_ROOT}/bin/conda" env create \
    --file "${PROJECT_ROOT}/environment/environment.yml"
fi

readonly ENV_PREFIX="${CONDA_ROOT}/envs/${ENV_NAME}"
"${CONDA_ROOT}/bin/conda" install --yes \
  --name "${ENV_NAME}" \
  "ffmpeg=7"

mkdir -p \
  "${ENV_PREFIX}/etc/conda/activate.d" \
  "${ENV_PREFIX}/etc/conda/deactivate.d"
install -m 0644 \
  "${PROJECT_ROOT}/environment/activate.d/piper-vla.sh" \
  "${ENV_PREFIX}/etc/conda/activate.d/piper-vla.sh"
install -m 0644 \
  "${PROJECT_ROOT}/environment/deactivate.d/piper-vla.sh" \
  "${ENV_PREFIX}/etc/conda/deactivate.d/piper-vla.sh"

if [[ ! -d "${LEROBOT_DIR}/.git" ]]; then
  mkdir -p "${PROJECT_ROOT}/third_party"
  git clone --branch v0.6.0 --depth 1 \
    https://github.com/huggingface/lerobot.git \
    "${LEROBOT_DIR}"
fi

actual_commit="$(git -C "${LEROBOT_DIR}" rev-parse HEAD)"
if [[ "${actual_commit}" != "${LEROBOT_COMMIT}" ]]; then
  echo "Unexpected LeRobot commit: ${actual_commit}" >&2
  exit 1
fi

readonly CONDA_RUN=("${CONDA_ROOT}/bin/conda" run --no-capture-output -n "${ENV_NAME}")

"${CONDA_RUN[@]}" python -m pip install \
  --upgrade \
  "pip<27" \
  "setuptools>=71,<81" \
  wheel

"${CONDA_RUN[@]}" python -m pip install \
  --retries 12 \
  --timeout 180 \
  "torch==2.11.0" \
  "torchvision==0.26.0" \
  --index-url "${PYTORCH_INDEX}"

"${CONDA_RUN[@]}" python -m pip install \
  --constraint "${PROJECT_ROOT}/environment/constraints.txt" \
  --editable "${LEROBOT_DIR}[core_scripts,training,smolvla]"

"${CONDA_RUN[@]}" python -m pip install \
  --constraint "${PROJECT_ROOT}/environment/constraints.txt" \
  "torchcodec==0.11.1" \
  "av==15.0.0" \
  "piper-sdk==0.6.1" \
  "python-can==4.6.1" \
  "pyorbbecsdk2==2.0.18"

"${CONDA_RUN[@]}" python -m pip install \
  --constraint "${PROJECT_ROOT}/environment/constraints.txt" \
  --editable "${PROJECT_ROOT}/plugins/lerobot_camera_orbbec"

"${CONDA_RUN[@]}" python -m pip install \
  --constraint "${PROJECT_ROOT}/environment/constraints.txt" \
  --editable "${PROJECT_ROOT}/plugins/lerobot_robot_piper" \
  --editable "${PROJECT_ROOT}/plugins/lerobot_robot_piper_active" \
  --editable "${PROJECT_ROOT}/plugins/lerobot_teleoperator_piper"

"${CONDA_RUN[@]}" python -m pip check
