#!/usr/bin/env bash
# Configures and builds the Controller subproject into Controller/build.
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cmake -S "$DIR/Controller" -B "$DIR/Controller/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$DIR/Controller/build" -j"$(nproc)"
