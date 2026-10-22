#!/usr/bin/env bash
# Runs the built controller executable. Run build_controller.sh first.
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# libpinocchio.so's URDF parser needs ROS Foxy's liburdfdom_model.so at
# runtime, and that lib dir isn't reachable via our binary's rpath (see
# Controller/CMakeLists.txt for why) or the system ldconfig cache.
export LD_LIBRARY_PATH="/opt/ros/foxy/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

exec "$DIR/Controller/build/controller" "$@"
