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
  | awk '
      /#egg=lerobot_camera_orbbec&subdirectory=plugins\/lerobot_camera_orbbec$/ {
        print "-e ../plugins/lerobot_camera_orbbec"
        next
      }
      /#egg=lerobot_robot_piper&subdirectory=plugins\/lerobot_robot_piper$/ {
        print "-e ../plugins/lerobot_robot_piper"
        next
      }
      /#egg=lerobot_robot_piper_active&subdirectory=plugins\/lerobot_robot_piper_active$/ {
        print "-e ../plugins/lerobot_robot_piper_active"
        next
      }
      /#egg=lerobot_teleoperator_piper&subdirectory=plugins\/lerobot_teleoperator_piper$/ {
        print "-e ../plugins/lerobot_teleoperator_piper"
        next
      }
      { print }
    ' \
  > "${ENV_DIR}/requirements.lock.txt"
python -m pip inspect \
  > "${ENV_DIR}/pip-inspect.json"

sha256sum \
  "${ENV_DIR}/conda-explicit-linux-64.txt" \
  "${ENV_DIR}/environment.resolved.yml" \
  "${ENV_DIR}/requirements.lock.txt" \
  "${ENV_DIR}/pip-inspect.json"
