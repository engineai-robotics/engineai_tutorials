#!/usr/bin/env bash
# Developer PC only. Download Ubuntu's emulator without installing host packages.
set -euo pipefail
replace=false
case ${1:-} in
 --replace) replace=true ;;
 --help) echo 'Usage: setup_arm64_builder.sh [--replace]'; exit 0 ;;
 '') ;;
 *) echo 'Usage: setup_arm64_builder.sh [--replace]' >&2; exit 2 ;;
esac
if [[ $(uname -m) == aarch64 || $(uname -m) == arm64 ]]; then
 echo 'Native ARM64 builder: no emulator registration required.'; exit 0
fi
source /etc/os-release
[[ $ID == ubuntu && $VERSION_ID == 22.04 && $(uname -m) == x86_64 ]] || {
 echo 'This helper supports Ubuntu 22.04 x86_64. Use a supported ARM64 builder on other systems.' >&2; exit 2;
}
docker info >/dev/null
if [[ -e /proc/sys/fs/binfmt_misc/qemu-aarch64 && $replace != true ]]; then
 echo 'ARM64 handler already exists; left unchanged. Use --replace only after checking other ARM64 workloads.'
 exit 0
fi
cache=${XDG_CACHE_HOME:-$HOME/.cache}/pm01_navigation/arm64-builder
mkdir -p "$cache"
temporary=$(mktemp -d "$cache/.prepare.XXXXXX")
trap 'rm -rf "$temporary"' EXIT
(
 cd "$temporary"
 apt-get download qemu-user-static
 dpkg-deb -x qemu-user-static*.deb root
)
install -m 755 "$temporary/root/usr/bin/qemu-aarch64-static" "$cache/qemu-aarch64-static"
registrar=tonistiigi/binfmt@sha256:1b804311fe87047a4c96d38b4b3ef6f62fca8cd125265917a9e3dc3c996c39e6
docker pull "$registrar"
if [[ -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ]]; then
 docker run --privileged --rm "$registrar" --uninstall qemu-aarch64
fi
docker run --privileged --rm \
 --mount "type=bind,src=$cache/qemu-aarch64-static,dst=/usr/bin/qemu-aarch64,readonly" \
 "$registrar" --install arm64
"$cache/qemu-aarch64-static" --version
echo 'ARM64 emulation ready. No ROS nodes or robot connections were started.'
