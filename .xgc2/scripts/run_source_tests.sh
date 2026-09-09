#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
work_dir="${RUNNER_TEMP:-/tmp}/xgc2-ugv-controller-compliance"
install_root="${RUNNER_TEMP:-/tmp}/xgc2-ugv-controller-install-root"

rm -rf "$work_dir" "$install_root"
mkdir -p "$work_dir/src/xgc2-ugv-controller"
rsync -a --delete "$REPO_ROOT/" "$work_dir/src/xgc2-ugv-controller/"

cd "$work_dir"
set +u
source /opt/ros/noetic/setup.bash
set -u
parallel_jobs="$(nproc)"
PYTHONPATH="$work_dir/src/xgc2-ugv-controller/unicycle_ugv_controller/tools" \
  python3 -B -m unittest discover \
    -s "$work_dir/src/xgc2-ugv-controller/unicycle_ugv_controller/tools/unicycle_nmpc/tests" \
    -v
PYTHONPATH="$work_dir/src/xgc2-ugv-controller/mecanum_ugv_controller/tools" \
  python3 -B -m unittest discover \
    -s "$work_dir/src/xgc2-ugv-controller/mecanum_ugv_controller/tools/holonomic_tracker/tests" \
    -v
# Exercise the optimized control loop used by runtime, retaining debug symbols
# for failures. The separate C++ quality job still builds the Debug profile.
catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" -DCMAKE_BUILD_TYPE=RelWithDebInfo
source devel/setup.bash
catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" \
  run_tests_unicycle_reference_trajectory \
  run_tests_unicycle_ugv_controller \
  run_tests_mecanum_ugv_controller \
  run_tests_ugv_reset_safety
catkin_test_results
DESTDIR="$install_root" catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" install \
  -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
  -DCMAKE_BUILD_TYPE=Release
test "$(rospack find unicycle_reference_trajectory)" = "$work_dir/src/xgc2-ugv-controller/unicycle_reference_trajectory"
test "$(rospack find unicycle_ugv_controller)" = "$work_dir/src/xgc2-ugv-controller/unicycle_ugv_controller"
test "$(rospack find mecanum_ugv_controller)" = "$work_dir/src/xgc2-ugv-controller/mecanum_ugv_controller"
roslaunch --files unicycle_reference_trajectory ugv_unicycle_reference_trajectory.launch >/tmp/xgc2-unicycle-reference-files.txt
roslaunch --files unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch >/tmp/xgc2-unicycle-controller-files.txt
roslaunch --files mecanum_ugv_controller ugv_mecanum_reset_controller.launch >/tmp/xgc2-mecanum-controller-files.txt
