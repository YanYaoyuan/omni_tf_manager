#!/usr/bin/env bash

# Assemble a relocatable Omni TF Manager overlay for an RDK S100 target.
# The target supplies RDK OS and ROS 2 Humble/TogetheROS; this archive carries
# the package executable, generated interfaces, profiles and package metadata.

set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 <install-root> <output-root>" >&2
  exit 2
fi

install_root=$(realpath "$1")
output_root=$(realpath -m "$2")

if [[ ! -d "${install_root}/lib" || ! -d "${install_root}/share" ]]; then
  echo "ERROR: invalid colcon install tree: ${install_root}" >&2
  exit 1
fi
if [[ ! -d "${install_root}/local" ]]; then
  echo "ERROR: generated ROSIDL Python tree is missing: ${install_root}/local" >&2
  exit 1
fi
case "${output_root}" in
  /|"${install_root}")
    echo "ERROR: unsafe runtime output path: ${output_root}" >&2
    exit 1
    ;;
esac

rm -rf "${output_root}"
mkdir -p \
  "${output_root}/bin" \
  "${output_root}/config/omni_tf_manager" \
  "${output_root}/lib" \
  "${output_root}/local" \
  "${output_root}/share"

cp -a "${install_root}/lib/." "${output_root}/lib/"
cp -a "${install_root}/local/." "${output_root}/local/"
cp -a "${install_root}/share/." "${output_root}/share/"
cp -a "${install_root}/share/omni_tf_manager/config/." \
  "${output_root}/config/omni_tf_manager/"

ln -sfn ../lib/omni_tf_manager/omni_tf_manager_node \
  "${output_root}/bin/omni_tf_manager_node"

cat > "${output_root}/setup.bash" <<'SETUP'
#!/usr/bin/env bash

_omni_tf_s100_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if [[ -f /opt/tros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/tros/humble/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[omni-tf-manager] ERROR: ROS 2 Humble/TogetheROS setup not found" >&2
  return 1 2>/dev/null || exit 1
fi

export PATH="${_omni_tf_s100_root}/bin:${PATH}"
export LD_LIBRARY_PATH="${_omni_tf_s100_root}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export AMENT_PREFIX_PATH="${_omni_tf_s100_root}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"

for _omni_tf_python_dir in \
  "${_omni_tf_s100_root}/lib/python3.10/dist-packages" \
  "${_omni_tf_s100_root}/lib/python3.10/site-packages" \
  "${_omni_tf_s100_root}/local/lib/python3.10/site-packages" \
  "${_omni_tf_s100_root}/local/lib/python3.10/dist-packages"
do
  if [[ -d "${_omni_tf_python_dir}" ]]; then
    export PYTHONPATH="${_omni_tf_python_dir}${PYTHONPATH:+:${PYTHONPATH}}"
  fi
done

unset _omni_tf_python_dir _omni_tf_s100_root
SETUP
chmod 755 "${output_root}/setup.bash"

cat > "${output_root}/README.txt" <<'README'
Omni TF Manager for RDK S100

This is a relocatable ROS 2 overlay for RDK OS/TogetheROS Humble.

Integrated Omni SLAM deployment:
  Use the omni_slam S100 bundle, which already contains this package. Do not
  start a second standalone omni_tf_manager process.

Standalone profile inspection/relay deployment:
  source setup.bash
  ros2 launch omni_tf_manager omni_tf_manager.launch.py \
    config_file:=$PWD/config/omni_tf_manager/omni_vbot_dog.yaml mode:=profile

The Vbot profile remains in shadow mode. It publishes reviewed canonical
LiDAR/IMU aliases but does not publish /tf or /tf_static.
README

node_path="${output_root}/lib/omni_tf_manager/omni_tf_manager_node"
python_package="${output_root}/local/lib/python3.10/dist-packages/omni_tf_manager"
profile="${output_root}/config/omni_tf_manager/omni_vbot_dog.yaml"

test -x "${node_path}"
if [[ ! -f "${output_root}/lib/libomni_tf_manager_core.so" && \
  ! -f "${output_root}/lib/libomni_tf_manager_core.a" ]]; then
  echo "ERROR: Omni TF Manager core library is missing" >&2
  exit 1
fi
test -f "${output_root}/share/omni_tf_manager/package.xml"
test -f "${profile}"
test -f "${python_package}/__init__.py"
find "${python_package}" -maxdepth 1 -type f \
  -name '*rosidl_typesupport_c*.so' -print -quit | grep -q .

grep -Fqx '    mode: shadow' "${profile}"
grep -Fqx '    sensor_relay.lidar.alias_verified: true' "${profile}"
grep -Fqx '    sensor_relay.lidar.input_frame: vita_lidar' "${profile}"
grep -Fqx '    sensor_relay.imu.alias_verified: true' "${profile}"
grep -Fqx '    sensor_relay.imu.input_frame: vita_lidar' "${profile}"

elf_count=0
while IFS= read -r -d '' runtime_file; do
  file_info=$(file "${runtime_file}")
  if grep -q ELF <<<"${file_info}"; then
    if ! grep -Eq 'ARM aarch64|ARM64' <<<"${file_info}"; then
      echo "ERROR: non-ARM64 ELF in S100 runtime: ${file_info}" >&2
      exit 1
    fi
    elf_count=$((elf_count + 1))
  fi
done < <(find "${output_root}" -type f -print0)

if (( elf_count == 0 )); then
  echo "ERROR: S100 runtime contains no ARM64 ELF files" >&2
  exit 1
fi

echo "Omni TF Manager S100 runtime ARM64 ELF files: ${elf_count}"
