#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PACKAGE="ros-${ROS_DISTRO}-xgc2-ugv-controller"
ROS_PACKAGES=(
  unicycle_reference_trajectory
  unicycle_ugv_controller
  mecanum_ugv_controller
  ugv_reset_safety
)

product_version() {
  # mawk (bionic) does not implement POSIX [[:space:]].
  awk '/^version:/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

if [[ -z "${VERSION}" ]]; then
  echo "package version is missing" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}"/*.deb

copy_path() {
  local src="$1"
  local dst_root="$2"
  if [[ -e "${src}" ]]; then
    mkdir -p "${dst_root}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${dst_root}${src#${INSTALL_ROOT}}"
  fi
}

pkg_root="${BUILD_DIR}/${PACKAGE}"
mkdir -p "${pkg_root}"

copy_ros_package() {
  local ros_package="$1"
  copy_path "${PREFIX_ROOT}/share/${ros_package}" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/${ros_package}" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/include/${ros_package}" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/pkgconfig/${ros_package}.pc" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/common-lisp/ros/${ros_package}" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/gennodejs/ros/${ros_package}" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/roseus/ros/${ros_package}" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/python3/dist-packages/${ros_package}" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/python2.7/dist-packages/${ros_package}" "${pkg_root}"
}

for ros_package in "${ROS_PACKAGES[@]}"; do
  copy_ros_package "${ros_package}"
done
copy_path "${PREFIX_ROOT}/lib/libunicycle_ugv_controller_nmpc_runtime.so" "${pkg_root}"
copy_path "${PREFIX_ROOT}/lib/libugv_reset_safety_math.so" "${pkg_root}"

mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${PACKAGE}"
cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: libeigen3-dev, libxgc2-state-machine-dev (>= 0.1.3-4~focal), libxgc2-math-dev (>= 0.5.6-6~focal), xgc2-acados (>= 0.1.0-10~focal), ros-${ROS_DISTRO}-xgc2-ros1-utils (>= 1.1.1-3), ros-${ROS_DISTRO}-xgc2-estimator-rigid-state-msgs (>= 1.2.0-3), ros-${ROS_DISTRO}-xgc2-unicycle-reference-trajectory-msgs (>= 1.3.0-13), ros-${ROS_DISTRO}-xgc2-geometry-msgs (>= 1.2.0-1), ros-${ROS_DISTRO}-message-runtime, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-rospy, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-nav-msgs
Replaces: ros-${ROS_DISTRO}-xgc2-controller (<< 1.3.2-1)
Breaks: ros-${ROS_DISTRO}-xgc2-controller (<< 1.3.2-1)
Description: XGC2 ROS1 UGV controller and reference trajectory packages
EOF
printf 'xgc2-ugv-controller package\n' > "${pkg_root}/usr/share/doc/${PACKAGE}/README"
chmod 0755 "${pkg_root}/DEBIAN"

fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
