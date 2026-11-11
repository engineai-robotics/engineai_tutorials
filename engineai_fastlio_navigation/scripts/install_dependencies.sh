#!/usr/bin/env bash
# Run on native Ubuntu 22.04 (sudo is used only for OS dependencies).
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
script_dir=$(cd "$(dirname "$0")" && pwd)
mode=${1:-sim}
case "$mode" in sim|hardware|desktop|core-client) ;; *) echo 'Usage: install_dependencies.sh [sim|hardware|desktop|core-client]' >&2; exit 2;; esac
packages=(build-essential cmake git pkg-config python3-colcon-common-extensions python3-pytest python3-numpy python3-yaml python3-pip
 ros-humble-ros-base ros-humble-rmw-fastrtps-cpp
 ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-ament-cmake-pytest ros-humble-ament-cmake-gtest libeigen3-dev)
while IFS= read -r pm01_nav2_package; do
 [[ -z $pm01_nav2_package ]] || packages+=("$pm01_nav2_package")
done < "$script_dir/config/nav2_packages.txt"
if [[ $mode == sim || $mode == hardware ]]; then
 packages+=(ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-xacro python3-matplotlib python3-dev libpcl-dev libyaml-cpp-dev libapr1-dev libasio-dev libtinyxml2-dev libceres-dev libsuitesparse-dev libboost-all-dev qtbase5-dev ros-humble-rviz-common ros-humble-rviz-default-plugins ros-humble-interactive-markers)
fi
if [[ $mode == sim || $mode == desktop ]]; then
 packages+=(ros-humble-rviz2 ros-humble-nav2-rviz-plugins python3-tk fonts-noto-cjk openssh-client rsync)
fi
if [[ $mode == sim ]]; then
 packages+=(mesa-utils x11-utils python3-pyqt5 ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-joy liburdfdom-dev)
fi
if [[ ${2:-} == --list-packages ]]; then
 printf '%s\n' "${packages[@]}"
 exit 0
fi
source /etc/os-release
[[ "$ID:$VERSION_ID" == ubuntu:22.04 ]] || { echo 'Supported OS: Ubuntu 22.04' >&2; exit 2; }
if [[ $mode == sim && $(uname -m) != x86_64 ]]; then echo 'Public walking-policy MNN libraries require x86_64' >&2; exit 2; fi
admin=(); [[ $EUID == 0 ]] || admin=(sudo)
ros_suite=${UBUNTU_CODENAME:-$VERSION_CODENAME}
ros_arch=$(dpkg --print-architecture)
# Repair only our known legacy duplicate before APT parses the source list.
# The helper uses bash/awk so it also works before Python is installed.
"${admin[@]}" bash "$script_dir/lib/ros_apt_source.sh" cleanup "$ros_suite" "$ros_arch"
"${admin[@]}" apt-get update
"${admin[@]}" apt-get install --no-remove -y ca-certificates curl gnupg locales software-properties-common
"${admin[@]}" add-apt-repository -y universe
if existing_ros_source=$("${admin[@]}" bash "$script_dir/lib/ros_apt_source.sh" exists "$ros_suite" "$ros_arch"); then
 printf 'Reusing ROS 2 APT source: %s\n' "$existing_ros_source"
else
 "${admin[@]}" install -d /usr/share/keyrings
 curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | "${admin[@]}" tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
 printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n' "$ros_arch" "$ros_suite" | "${admin[@]}" tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi
"${admin[@]}" apt-get update
"${admin[@]}" apt-get install --no-install-recommends --no-remove -y "${packages[@]}"
# Dependencies above are explicit. Do not initialize or update a global rosdep cache.
printf 'Dependencies installed. Next: bash scripts/build.sh %s\n' "$mode"
