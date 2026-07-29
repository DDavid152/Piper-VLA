#!/usr/bin/env bash
set -euo pipefail

readonly CONDA_ROOT="/home/ubuntu22/miniforge3"
readonly MINIFORGE_VERSION="26.3.2-3"
readonly INSTALLER_NAME="Miniforge3-${MINIFORGE_VERSION}-Linux-x86_64.sh"
readonly INSTALLER_URL="https://github.com/conda-forge/miniforge/releases/download/${MINIFORGE_VERSION}/${INSTALLER_NAME}"
readonly INSTALLER_SHA256="848194851a98903134187fbb4ab50efe87b003e0c0f808f97644b7524a62bf2c"

if [[ -x "${CONDA_ROOT}/bin/conda" ]]; then
  echo "Miniforge is already installed at ${CONDA_ROOT}."
  "${CONDA_ROOT}/bin/conda" --version
  exit 0
fi

if [[ -e "${CONDA_ROOT}" ]]; then
  echo "${CONDA_ROOT} exists but is not a usable Miniforge installation." >&2
  exit 1
fi

tmp_dir="$(mktemp -d)"
trap 'rm -rf -- "${tmp_dir}"' EXIT
readonly installer="${tmp_dir}/${INSTALLER_NAME}"

curl -fsSL --retry 5 --retry-all-errors \
  "${INSTALLER_URL}" \
  --output "${installer}"

printf '%s  %s\n' "${INSTALLER_SHA256}" "${installer}" \
  | sha256sum --check -

bash "${installer}" -b -p "${CONDA_ROOT}"
"${CONDA_ROOT}/bin/conda" config --system \
  --set auto_activate_base false
"${CONDA_ROOT}/bin/conda" --version
