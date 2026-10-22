#!/usr/bin/env bash
# Configures and builds the Mujoco subproject into Mujoco/build.
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cmake -S "$DIR/Mujoco" -B "$DIR/Mujoco/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$DIR/Mujoco/build" -j"$(nproc)"
