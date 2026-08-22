#!/usr/bin/env bash

# Reproducible RDK S100 cross build for Omni TF Manager.
# All generated source, sysroot and colcon output live below S100_WORK_ROOT.

set -euo pipefail

report_error() {
  local exit_code=$?
  printf '[s100-cross] ERROR: line %s: %s (exit %s)\n' \
    "$1" "$2" "${exit_code}" >&2
  exit "${exit_code}"
}

trap 'report_error "${LINENO}" "${BASH_COMMAND}"' ERR

SOURCE_ROOT=${S100_SOURCE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
WORK_ROOT=${S100_WORK_ROOT:-${RUNNER_TEMP:-/tmp}/s100-cross}
S100_IMAGE=${S100_IMAGE:-pc_tros_ubuntu22.04:v1.0.0}
ROBOT_DEV_CONFIG_REF=${ROBOT_DEV_CONFIG_REF:-f44b1ad2575b34c189fd470de1db9c5ad48b6886}
S100_SYSROOT_REF=${S100_SYSROOT_REF:-de9fa286f71a72d24c349477cd59f41cc2cc3d8f}
S100_CLEAN=${S100_CLEAN:-1}

log() {
  printf '[s100-cross] %s\n' "$*"
}

die() {
  printf '[s100-cross] ERROR: %s\n' "$*" >&2
  exit 1
}

for command_name in docker git python3 rsync; do
  command -v "${command_name}" >/dev/null 2>&1 || die "missing command: ${command_name}"
done

SOURCE_ROOT=$(realpath -m "${SOURCE_ROOT}")
WORK_ROOT=$(realpath -m "${WORK_ROOT}")

[[ -f "${SOURCE_ROOT}/package.xml" ]] || \
  die "invalid S100_SOURCE_ROOT: ${SOURCE_ROOT}"

case "${WORK_ROOT}" in
  /|/data|/home|"${SOURCE_ROOT}")
    die "unsafe S100_WORK_ROOT: ${WORK_ROOT}"
    ;;
esac

TROS_ROOT="${WORK_ROOT}/cc_ws/tros_ws"
ROBOT_DEV_CONFIG_DIR="${TROS_ROOT}/robot_dev_config"

if [[ "${S100_CLEAN}" == "1" ]]; then
  log "cleaning generated workspace: ${WORK_ROOT}/cc_ws"
  rm -rf "${WORK_ROOT}/cc_ws"
fi

mkdir -p "${TROS_ROOT}/src/omni_tf_manager"

log "cloning robot_dev_config at ${ROBOT_DEV_CONFIG_REF}"
git clone --filter=blob:none \
  https://github.com/D-Robotics/robot_dev_config.git \
  "${ROBOT_DEV_CONFIG_DIR}"
git -C "${ROBOT_DEV_CONFIG_DIR}" checkout --detach "${ROBOT_DEV_CONFIG_REF}"

log "copying Omni TF Manager source into the cross workspace"
rsync -a --delete \
  --exclude='/.git/' \
  --exclude='/build/' \
  --exclude='/install/' \
  --exclude='/log/' \
  "${SOURCE_ROOT}/" \
  "${TROS_ROOT}/src/omni_tf_manager/"

log "starting toolchain container"
docker run --rm -i \
  --network host \
  -e "S100_SYSROOT_REF=${S100_SYSROOT_REF}" \
  -e "HTTP_PROXY=${S100_CONTAINER_HTTP_PROXY:-${HTTP_PROXY:-}}" \
  -e "HTTPS_PROXY=${S100_CONTAINER_HTTPS_PROXY:-${HTTPS_PROXY:-}}" \
  -e "NO_PROXY=${NO_PROXY:-}" \
  -v "${WORK_ROOT}:/mnt/s100-cross" \
  -w /mnt/s100-cross/cc_ws/tros_ws \
  "${S100_IMAGE}" \
  bash -s <<'CONTAINER'
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
report_container_error() {
  local exit_code=$?
  printf '[s100-container] ERROR: line %s: %s (exit %s)\n' \
    "$1" "$2" "${exit_code}" >&2
  exit "${exit_code}"
}

trap 'report_container_error "${LINENO}" "${BASH_COMMAND}"' ERR

echo "=== Check toolchain and workspace ==="
for required_file in \
  robot_dev_config/build.sh \
  robot_dev_config/s100_build.sh \
  robot_dev_config/aarch64_toolchainfile.cmake \
  robot_dev_config/ros2.repos; do
  if [[ ! -f "${required_file}" ]]; then
    echo "ERROR: missing required toolchain file: ${required_file}" >&2
    exit 1
  fi
done

shallow_checkout() {
  local url=$1
  local ref=$2
  local target=$3
  local attempt

  mkdir -p "${target}"
  git -C "${target}" init --quiet
  git -C "${target}" remote add origin "${url}"

  # Force HTTP/1.1 because the local proxy corrupted a long HTTP/2 TLS stream.
  # Low-speed detection prevents an otherwise healthy process from hanging
  # forever if the direct or proxied connection stops delivering bytes.
  for attempt in 1 2 3; do
    if git -C "${target}" \
      -c http.version=HTTP/1.1 \
      -c http.lowSpeedLimit=1024 \
      -c http.lowSpeedTime=60 \
      fetch --progress --no-tags --depth=1 origin "${ref}"; then
      break
    fi
    if (( attempt == 3 )); then
      echo "ERROR: failed to fetch ${url} at ${ref} after ${attempt} attempts" >&2
      return 1
    fi
    echo "Retrying ${url} (${attempt}/3)" >&2
  done
  git -C "${target}" checkout --quiet --detach FETCH_HEAD
}

checkout_sysroot_in_batches() {
  local ref=$1
  local cache=/mnt/s100-cross/cache/sysroot_docker-${ref}
  local complete_marker=${cache}/.usr_s100-complete
  local marker_dir=${cache}/.git/s100-completed-chunks
  local chunk_dir
  local oid_dir
  local chunk
  local chunk_name
  local export_index
  local export_root
  local oid_chunk
  local oid_part
  local oid_part_name
  local attempt
  local completed=0
  local missing_count
  local total=0

  mkdir -p "${cache}"
  if [[ ! -d "${cache}/.git" ]]; then
    git -C "${cache}" init --quiet
    git -C "${cache}" remote add origin \
      https://github.com/D-Robotics/sysroot_docker.git
  fi

  git -C "${cache}" config http.version HTTP/1.1
  git -C "${cache}" config http.lowSpeedLimit 1024
  git -C "${cache}" config http.lowSpeedTime 60
  git -C "${cache}" config gc.auto 0
  # An interrupted container can leave this transient lock behind. Builds are
  # serialized by test_s100_local.sh, so no other process can own it here.
  rm -f "${cache}/.git/index.lock"

  if ! git -C "${cache}" cat-file -e "${ref}^{commit}" 2>/dev/null; then
    for attempt in 1 2 3; do
      if git -C "${cache}" fetch \
        --progress --no-tags --depth=1 --filter=blob:none origin "${ref}"; then
        break
      fi
      if (( attempt == 3 )); then
        echo "ERROR: failed to fetch sysroot metadata after ${attempt} attempts" >&2
        return 1
      fi
      echo "Retrying sysroot metadata (${attempt}/3)" >&2
    done
  fi

  git -C "${cache}" config remote.origin.promisor true
  git -C "${cache}" config remote.origin.partialclonefilter blob:none
  git -C "${cache}" update-ref refs/heads/s100-cache "${ref}"
  git -C "${cache}" symbolic-ref HEAD refs/heads/s100-cache

  if [[ -f "${complete_marker}" ]] && \
    [[ $(<"${complete_marker}") == "${ref}" ]]; then
    echo "Reusing complete sysroot cache: ${cache}"
  else
    mkdir -p "${marker_dir}"
    chunk_dir=$(mktemp -d /tmp/s100-sysroot-chunks.XXXXXX)
    oid_dir=$(mktemp -d /tmp/s100-sysroot-oids.XXXXXX)

    git -C "${cache}" ls-tree -r --name-only "${ref}" usr_s100 \
      | sed 's|^|/|' \
      | split -l 2000 -d -a 4 - "${chunk_dir}/chunk-"
    git -C "${cache}" ls-tree -r "${ref}" usr_s100 \
      | awk '{print $3}' \
      | split -l 2000 -d -a 4 - "${oid_dir}/chunk-"

    total=$(find "${chunk_dir}" -maxdepth 1 -type f -name 'chunk-*' | wc -l)

    for chunk in "${chunk_dir}"/chunk-*; do
      chunk_name=$(basename "${chunk}")
      if [[ -f "${marker_dir}/${chunk_name}" ]]; then
        completed=$((completed + 1))
        continue
      fi

      # A single checkout can request hundreds of MiB and is prone to being
      # dropped by proxies. Ask Git's partial-clone promisor for small groups
      # of blobs first, and leave per-group markers so a retry resumes.
      oid_chunk="${oid_dir}/${chunk_name}"
      split -l 250 -d -a 4 "${oid_chunk}" "${oid_chunk}.part-"
      for oid_part in "${oid_chunk}".part-*; do
        oid_part_name=$(basename "${oid_part}")
        if [[ -f "${marker_dir}/${oid_part_name}" ]]; then
          continue
        fi

        for attempt in 1 2 3; do
          if git -C "${cache}" \
              -c fetch.negotiationAlgorithm=noop \
              -c http.version=HTTP/1.1 \
              fetch origin \
              --no-tags \
              --no-write-fetch-head \
              --recurse-submodules=no \
              --filter=blob:none \
              --stdin < "${oid_part}"; then
            break
          fi
          if (( attempt == 3 )); then
            echo "ERROR: sysroot blobs ${oid_part_name} failed after ${attempt} attempts" >&2
            return 1
          fi
          echo "Retrying sysroot blobs ${oid_part_name} (${attempt}/3)" >&2
        done
        touch "${marker_dir}/${oid_part_name}"
      done

      touch "${marker_dir}/${chunk_name}"
      completed=$((completed + 1))
      echo "sysroot chunks: ${completed}/${total}"
    done

    missing_count=$(git -C "${cache}" rev-list \
      --objects --missing=print "${ref}" -- usr_s100 \
      | grep -c '^?' || true)
    if (( missing_count != 0 )); then
      echo "ERROR: sysroot object cache is missing ${missing_count} blobs" >&2
      return 1
    fi
    printf '%s\n' "${ref}" > "${complete_marker}"
  fi

  test "$(git -C "${cache}" rev-parse HEAD)" = "${ref}"
  mkdir -p ../sysroot_docker
  export_root=$(realpath ../sysroot_docker)
  mkdir -p "${export_root}/usr_s100"
  export_index=$(mktemp /tmp/s100-sysroot-index.XXXXXX)
  rm -f "${export_index}"
  GIT_INDEX_FILE="${export_index}" \
    git -C "${cache}" read-tree "${ref}:usr_s100"
  GIT_INDEX_FILE="${export_index}" \
    git -C "${cache}" checkout-index \
      --all --force --prefix="${export_root}/usr_s100/"
  rm -f "${export_index}"
  test -d ../sysroot_docker/usr_s100
  printf '%s\n' "${ref}" > ../sysroot_docker/.s100-sysroot-ref
}

echo "=== Fetch required ROS/TROS dependencies ==="
checkout_sysroot_in_batches "${S100_SYSROOT_REF}"
shallow_checkout \
  https://github.com/D-Robotics/tros_arm_build.git \
  develop \
  tros_arm_build

echo "=== Verify pinned S100 sysroot ==="
test -d ../sysroot_docker/usr_s100
actual_sysroot_ref=$(<../sysroot_docker/.s100-sysroot-ref)
test "${actual_sysroot_ref}" = "${S100_SYSROOT_REF}"
echo "sysroot revision: ${actual_sysroot_ref}"

echo "=== Verify official TROS build environment ==="
for required_command in \
  aarch64-linux-gnu-gcc \
  aarch64-linux-gnu-g++ \
  cmake \
  colcon \
  curl \
  file \
  readelf; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    echo "ERROR: official TROS image is missing: ${required_command}" >&2
    exit 1
  fi
done
aarch64-linux-gnu-gcc --version
test -f /opt/ros/humble/setup.bash

echo "=== Activate ROS 2 build environment ==="
set +u
source /opt/ros/humble/setup.bash
set -u

echo "=== Build S100 ==="
bash robot_dev_config/build.sh \
  -p S100 \
  -r '--packages-up-to omni_tf_manager'

# robot_dev_config/build.sh swallows the colcon exit status. These checks make
# the shared script fail if the real target outputs were not produced.
test -x install/lib/omni_tf_manager/omni_tf_manager_node
test -f install/share/omni_tf_manager/config/generic_four_sensor.yaml
test -f install/share/omni_tf_manager/config/omni_dog.yaml
test -f install/share/omni_tf_manager/config/omni_vbot_dog.yaml

echo "=== Verify output architecture ==="
elf_count=0
while IFS= read -r file_path; do
  file_info=$(file "${file_path}")
  if grep -q ELF <<<"${file_info}"; then
    echo "${file_info}"
    grep -Eq 'ARM aarch64|ARM64' <<<"${file_info}"
    elf_count=$((elf_count + 1))
  fi
done < <(find install -type f -perm /111)

if (( elf_count == 0 )); then
  echo 'ERROR: no executable ARM64 ELF files were produced' >&2
  exit 1
fi

du -sh install
echo "S100 cross build completed"
CONTAINER

log "build output: ${TROS_ROOT}/install"


