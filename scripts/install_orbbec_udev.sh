#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script as root: sudo $0" >&2
  exit 1
fi

readonly RULE_URL="https://raw.githubusercontent.com/orbbec/pyorbbecsdk/v2.0.18/scripts/env_setup/99-obsensor-libusb.rules"
readonly RULE_SHA256="6d6c11e4ad46f5976f4220f733c39c07b7ef469fffa0aa8d75ae7cb4e0385847"
readonly RULE_TARGET="/etc/udev/rules.d/99-obsensor-libusb.rules"

tmp_dir="$(mktemp -d)"
trap 'rm -rf -- "${tmp_dir}"' EXIT

curl -fsSL --retry 5 --retry-all-errors \
  "${RULE_URL}" \
  --output "${tmp_dir}/99-obsensor-libusb.rules"

printf '%s  %s\n' \
  "${RULE_SHA256}" \
  "${tmp_dir}/99-obsensor-libusb.rules" \
  | sha256sum --check -

install -m 0644 "${tmp_dir}/99-obsensor-libusb.rules" "${RULE_TARGET}"
udevadm control --reload
udevadm trigger

echo "Installed ${RULE_TARGET} from pyorbbecsdk v2.0.18."
