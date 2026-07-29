#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script as root: sudo $0" >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y \
  build-essential \
  cmake \
  ninja-build \
  pkg-config \
  python3-dev \
  git \
  git-lfs \
  curl \
  wget \
  ffmpeg \
  can-utils \
  ethtool \
  usbutils \
  v4l-utils \
  jq \
  libgl1 \
  libglib2.0-0 \
  libusb-1.0-0 \
  libudev1
