#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s "ros-${ROS_DISTRO}-xgc2-ugv-controller" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-estimator-rigid-state-msgs" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-unicycle-reference-trajectory-msgs" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-ros1-utils" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-geometry-msgs" >/dev/null
dpkg -s libxgc2-state-machine-dev >/dev/null
dpkg -s libxgc2-math-dev >/dev/null
dpkg -s xgc2-acados >/dev/null
xgc2_acados_version="$(dpkg-query -W -f='${Version}' xgc2-acados)"
dpkg --compare-versions "${xgc2_acados_version}" ge "0.1.0-10~focal"
test "$(rospack find unicycle_reference_trajectory)" = "/opt/ros/${ROS_DISTRO}/share/unicycle_reference_trajectory"
test "$(rospack find unicycle_ugv_controller)" = "/opt/ros/${ROS_DISTRO}/share/unicycle_ugv_controller"
test "$(rospack find mecanum_ugv_controller)" = "/opt/ros/${ROS_DISTRO}/share/mecanum_ugv_controller"
test "$(rospack find rigid_state_estimator_msgs)" = "/opt/ros/${ROS_DISTRO}/share/rigid_state_estimator_msgs"
test "$(rospack find unicycle_reference_trajectory_msgs)" = "/opt/ros/${ROS_DISTRO}/share/unicycle_reference_trajectory_msgs"
rosmsg show rigid_state_estimator_msgs/RigidStateEstimate | grep -q '^uint8 estimator_state$'
rosmsg show rigid_state_estimator_msgs/RigidStateEstimate | grep -q '^geometry_msgs/Vector3 angular_velocity$'
rosmsg show unicycle_reference_trajectory_msgs/PlanarPvaReference | grep -q '^geometry_msgs/Point position$'
rosmsg show unicycle_reference_trajectory_msgs/AnalyticReference | grep -q '^uint16 analytic_type$'
rosmsg show unicycle_reference_trajectory_msgs/SampledReference | grep -q '^unicycle_reference_trajectory_msgs/PlanarReferencePoint\[\] points$'
test -f "/opt/ros/${ROS_DISTRO}/share/unicycle_reference_trajectory/config/unicycle_reference_trajectory.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/unicycle_reference_trajectory/launch/ugv_unicycle_reference_trajectory.launch"
test -f "/opt/ros/${ROS_DISTRO}/include/unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"
test -f "/opt/ros/${ROS_DISTRO}/include/unicycle_reference_trajectory_msgs/AnalyticReference.h"
test -f "/opt/ros/${ROS_DISTRO}/include/unicycle_reference_trajectory_msgs/PlanarReferencePoint.h"
test -f "/opt/ros/${ROS_DISTRO}/share/unicycle_ugv_controller/config/unicycle_ugv_controller.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/unicycle_ugv_controller/launch/ugv_unicycle_nmpc_controller.launch"
test -f "/opt/ros/${ROS_DISTRO}/include/unicycle_ugv_controller/unicycle_ugv_controller.h"
test -x "/opt/ros/${ROS_DISTRO}/lib/unicycle_ugv_controller/unicycle_ugv_controller_node"
test -f "/opt/ros/${ROS_DISTRO}/lib/libunicycle_ugv_controller_nmpc_runtime.so"
test -f "/opt/ros/${ROS_DISTRO}/share/mecanum_ugv_controller/config/mecanum_ugv_controller.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/mecanum_ugv_controller/launch/ugv_mecanum_reset_controller.launch"
test -x "/opt/ros/${ROS_DISTRO}/lib/mecanum_ugv_controller/mecanum_ugv_controller_node"
roslaunch --files unicycle_reference_trajectory ugv_unicycle_reference_trajectory.launch >/tmp/xgc2-unicycle-reference-files.txt
roslaunch --files unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch >/tmp/xgc2-unicycle-controller-files.txt
roslaunch --files mecanum_ugv_controller ugv_mecanum_reset_controller.launch >/tmp/xgc2-mecanum-controller-files.txt

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find "/opt/ros/${ROS_DISTRO}/lib/unicycle_ugv_controller" \
  "/opt/ros/${ROS_DISTRO}/lib/unicycle_reference_trajectory" \
  "/opt/ros/${ROS_DISTRO}/lib/mecanum_ugv_controller" \
  "/opt/ros/${ROS_DISTRO}/lib/ugv_reset_safety" \
  "/opt/ros/${ROS_DISTRO}/lib/libugv_reset_safety_math.so" \
  "/opt/ros/${ROS_DISTRO}/lib/libunicycle_ugv_controller_nmpc_runtime.so" -type f 2>/dev/null | sort -u)

test -x "/opt/ros/${ROS_DISTRO}/lib/ugv_reset_safety/ugv_reset_coordinator_node"
rosmsg show ugv_reset_safety/ResetRequest | grep -q "^uint32 generation$"
rosmsg show ugv_reset_safety/ResetResponse | grep -q "^uint8 status$"
test "$(rospack find ugv_reset_safety)" = "/opt/ros/${ROS_DISTRO}/share/ugv_reset_safety"
roslaunch --files ugv_reset_safety ugv_reset_coordinator.launch fleet_config:="/opt/ros/${ROS_DISTRO}/share/ugv_reset_safety/config/mixed_pair.yaml" >/tmp/xgc2-reset-coordinator-files.txt

# Exercise the installed public interface using only its exported compiler flags.
# A missing Eigen include export or pkg-config dependency must fail this check.
test "$(pkg-config --variable=prefix ugv_reset_safety)" = "/opt/ros/${ROS_DISTRO}"
reset_public_cflags_text="$(pkg-config --cflags ugv_reset_safety)"
read -r -a reset_public_cflags <<< "${reset_public_cflags_text}"
c++ -std=c++17 -fsyntax-only -x c++ "${reset_public_cflags[@]}" - <<'CPP'
#include <ugv_reset_safety/fleet_guidance.h>
#include <ugv_reset_safety/fleet_schedule.h>
#include <ugv_reset_safety/reset_client.h>
#include <ugv_reset_safety/reset_guidance.h>
#include <ugv_reset_safety/reset_session.h>
#include <ugv_reset_safety/safety_filter.h>
int main() { return 0; }
CPP

reset_osqp_path="$(ldd "/opt/ros/${ROS_DISTRO}/lib/libugv_reset_safety_math.so" |
  awk '$1 == "libosqp.so" {print $3; exit}')"
if [[ -z "${reset_osqp_path}" ]] ||
   [[ "$(readlink -f "${reset_osqp_path}")" != "$(readlink -f /opt/xgc2/acados/lib/libosqp.so)" ]]; then
  echo "Reset safety library resolved an unexpected OSQP: ${reset_osqp_path}" >&2
  exit 1
fi
echo "Installed package check passed"
