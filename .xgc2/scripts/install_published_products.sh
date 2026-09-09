#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export DEBIAN_FRONTEND=noninteractive
ROS_DISTRO="${ROS_DISTRO:-noetic}"
if [[ -d "${XGC2_LOCAL_DEB_DIR:-}" ]]; then
  shopt -s nullglob
  local_debs=("${XGC2_LOCAL_DEB_DIR}"/*.deb)
  shopt -u nullglob
  if [[ ${#local_debs[@]} -gt 0 ]]; then
    dpkg -i "${local_debs[@]}" || apt-get -f install -y --no-install-recommends
  fi
fi
"${SCRIPT_DIR}/setup_xgc2_apt_source.sh"
apt-get install -y --no-install-recommends \
  libxgc2-math-dev \
  libxgc2-state-machine-dev \
  xgc2-acados \
  "ros-${ROS_DISTRO}-xgc2-estimator-rigid-state-msgs" \
  "ros-${ROS_DISTRO}-xgc2-unicycle-reference-trajectory-msgs" \
  "ros-${ROS_DISTRO}-xgc2-ros1-utils" \
  "ros-${ROS_DISTRO}-xgc2-geometry-msgs"
