#!/usr/bin/env bash
set -euo pipefail

readonly PROJECT_ROOT="/home/ubuntu22/Piper-VLA"
readonly CONDA_ROOT="/home/ubuntu22/miniforge3"
readonly ENV_NAME="lerobot"
readonly ENV_DIR="${PROJECT_ROOT}/environment"

source "${CONDA_ROOT}/etc/profile.d/conda.sh"
conda activate "${ENV_NAME}"

conda list --explicit \
  > "${ENV_DIR}/conda-explicit-linux-64.txt"
conda env export --no-builds \
  | sed '/^prefix:/d' \
  > "${ENV_DIR}/environment.resolved.yml"
python -m pip freeze --all --exclude pip \
  > "${ENV_DIR}/requirements.lock.txt"
python -m pip inspect \
  > "${ENV_DIR}/pip-inspect.json"

sha256sum \
  "${ENV_DIR}/conda-explicit-linux-64.txt" \
  "${ENV_DIR}/environment.resolved.yml" \
  "${ENV_DIR}/requirements.lock.txt" \
  "${ENV_DIR}/pip-inspect.json"
