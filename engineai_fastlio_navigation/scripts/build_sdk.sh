#!/usr/bin/env bash
# Shared by the native build and the cached Docker SDK layer.
set -euo pipefail
sdk=${1:?SDK source directory required}
prefix=${2:?Installation prefix required}
[[ -f "$sdk/CMakeLists.txt" ]] || sdk="$sdk/source"
sdk=$(cd "$sdk" && pwd)
# CMake caches embed the old source path. Retire only this generated cache
# when switching from the former full upstream checkout to the asset tree.
cache=.deps/sdk-build/CMakeCache.txt
if [[ -f "$cache" ]]; then
  previous=$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$cache")
  if [[ -n "$previous" && "$previous" != "$sdk" ]]; then
    retired=$(mktemp -d .deps/retired-sdk-build.XXXXXX)
    mv .deps/sdk-build "$retired/"
  fi
fi
export CMAKE_BUILD_PARALLEL_LEVEL=${PM01_BUILD_JOBS:-2}
export MAKEFLAGS="-j${CMAKE_BUILD_PARALLEL_LEVEL}"
cmake -S "$sdk" -B .deps/sdk-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix"
cmake --build .deps/sdk-build --parallel "$CMAKE_BUILD_PARALLEL_LEVEL"
cmake --install .deps/sdk-build
