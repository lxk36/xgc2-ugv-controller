#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"

require_command() {
  local command_name="$1"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "missing required command: ${command_name}" >&2
    exit 1
  fi
}

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "missing ROS setup: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

require_command clang-format
require_command clang-tidy
require_command catkin_make
require_command git
require_command rsync

if [[ "${GITHUB_ACTIONS:-}" == "true" ]]; then
  safe_directory_configured=false
  while IFS= read -r safe_directory; do
    if [[ "${safe_directory}" == "${REPO_ROOT}" ]]; then
      safe_directory_configured=true
      break
    fi
  done < <(git config --global --get-all safe.directory || true)
  if [[ "${safe_directory_configured}" != "true" ]]; then
    git config --global --add safe.directory "${REPO_ROOT}"
  fi
fi

if [[ -n "${XGC2_CLANG_TIDY_SCOPE:-}" ]]; then
  TIDY_SCOPE="${XGC2_CLANG_TIDY_SCOPE}"
elif [[ "${GITHUB_EVENT_NAME:-}" == "push" ]]; then
  TIDY_SCOPE="changed"
else
  TIDY_SCOPE="full"
fi

mapfile -d '' CXX_FILES < <(
  cd "${REPO_ROOT}"
  find unicycle_ugv_controller/include unicycle_ugv_controller/src unicycle_ugv_controller/test \
       unicycle_reference_trajectory/include unicycle_reference_trajectory/src unicycle_reference_trajectory/test \
       mecanum_ugv_controller/include mecanum_ugv_controller/src mecanum_ugv_controller/test \
       ugv_reset_safety/include ugv_reset_safety/src ugv_reset_safety/test \
    -type f \
    \( -name "*.cpp" -o -name "*.cc" -o -name "*.cxx" -o \
      -name "*.h" -o -name "*.hpp" -o -name "*.hh" -o -name "*.hxx" \) \
    -print0 | sort -z
)

if [[ "${#CXX_FILES[@]}" -eq 0 ]]; then
  echo "no C++ files found" >&2
  exit 1
fi

echo "Running clang-format..."
(
  cd "${REPO_ROOT}"
  clang-format --dry-run --Werror "${CXX_FILES[@]}"
)

collect_full_tidy_files() {
  mapfile -d '' TIDY_REL_FILES < <(
    cd "${REPO_ROOT}"
    find unicycle_ugv_controller/src unicycle_ugv_controller/test \
         unicycle_reference_trajectory/src unicycle_reference_trajectory/test \
         mecanum_ugv_controller/src mecanum_ugv_controller/test \
         ugv_reset_safety/src ugv_reset_safety/test \
      -type f \( -name "*.cpp" -o -name "*.cc" -o -name "*.cxx" \) \
      -print0 | sort -z
  )
}

TIDY_REL_FILES=()
case "${TIDY_SCOPE}" in
  full)
    collect_full_tidy_files
    ;;
  changed)
    if ! git -C "${REPO_ROOT}" rev-parse --verify HEAD^ >/dev/null 2>&1; then
      git_ref_name="${GITHUB_REF_NAME:-}"
      if [[ -n "${git_ref_name}" ]]; then
        timeout 30s git -C "${REPO_ROOT}" fetch --depth=2 origin "${git_ref_name}" || true
      fi
    fi

    if ! git -C "${REPO_ROOT}" rev-parse --verify HEAD^ >/dev/null 2>&1; then
      echo "Previous commit unavailable after fetch; falling back to full clang-tidy scope"
      collect_full_tidy_files
    else
      header_changed=false
      while IFS= read -r -d '' changed_path; do
        case "${changed_path}" in
          unicycle_ugv_controller/include/*|unicycle_ugv_controller/src/*|unicycle_ugv_controller/test/*|unicycle_reference_trajectory/include/*|unicycle_reference_trajectory/src/*|unicycle_reference_trajectory/test/*|mecanum_ugv_controller/include/*|mecanum_ugv_controller/src/*|mecanum_ugv_controller/test/*|ugv_reset_safety/include/*|ugv_reset_safety/src/*|ugv_reset_safety/test/*)
            case "${changed_path}" in
              *.h|*.hpp|*.hh|*.hxx)
                header_changed=true
                ;;
              *.cpp|*.cc|*.cxx)
                case "${changed_path}" in
                  unicycle_ugv_controller/src/*|unicycle_ugv_controller/test/*|unicycle_reference_trajectory/src/*|unicycle_reference_trajectory/test/*|mecanum_ugv_controller/include/*|mecanum_ugv_controller/src/*|mecanum_ugv_controller/test/*|ugv_reset_safety/include/*|ugv_reset_safety/src/*|ugv_reset_safety/test/*)
                    TIDY_REL_FILES+=("${changed_path}")
                    ;;
                esac
                ;;
            esac
            ;;
        esac
      done < <(git -C "${REPO_ROOT}" diff --name-only -z HEAD^ HEAD --)

      if [[ "${header_changed}" == "true" ]]; then
        echo "Header changes detected; using full clang-tidy scope"
        collect_full_tidy_files
      fi
    fi
    ;;
  *)
    echo "invalid XGC2_CLANG_TIDY_SCOPE: ${TIDY_SCOPE}" >&2
    exit 1
    ;;
esac

if [[ "${#TIDY_REL_FILES[@]}" -eq 0 ]]; then
  echo "No clang-tidy translation units selected for scope: ${TIDY_SCOPE}"
  echo "C++ quality check passed"
  exit 0
fi

WORK_DIR="${RUNNER_TEMP:-${TMPDIR:-/tmp}}/xgc2-ugv-controller-cpp-quality"
rm -rf "${WORK_DIR}"
mkdir -p "${WORK_DIR}/src/xgc2-ugv-controller"
rsync -a --delete --exclude ".git" "${REPO_ROOT}/" "${WORK_DIR}/src/xgc2-ugv-controller/"

echo "Generating compile_commands.json..."
(
  cd "${WORK_DIR}"
  parallel_jobs="$(nproc)"
  catkin_make \
    -j"${parallel_jobs}" \
    -l"${parallel_jobs}" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=Debug
)

echo "Running clang-tidy..."
TIDY_FILES=()
for tidy_rel_file in "${TIDY_REL_FILES[@]}"; do
  TIDY_FILES+=("${WORK_DIR}/src/xgc2-ugv-controller/${tidy_rel_file}")
done
echo "Selected ${#TIDY_FILES[@]} clang-tidy translation units for scope: ${TIDY_SCOPE}"
printf '%s\0' "${TIDY_FILES[@]}" | xargs -0 -n 1 -P "$(nproc)" clang-tidy \
  -p "${WORK_DIR}/build" \
  -header-filter="^${WORK_DIR}/src/xgc2-ugv-controller/(unicycle_ugv_controller|unicycle_reference_trajectory|mecanum_ugv_controller|ugv_reset_safety)/(src|test)/" \
  -quiet

echo "C++ quality check passed"
