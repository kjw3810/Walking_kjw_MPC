#!/usr/bin/env bash
# Opens the MuJoCo interactive viewer (simulate), built from vendored source
# (Mujoco/mujoco_engine), on a model.
# Usage: ./run_mujoco.sh [path/to/model.xml]   (defaults to the pendulum test model)
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODEL="${1:-$DIR/Mujoco/model/pendulum.xml}"

exec "$DIR/Mujoco/build/bin/simulate" "$MODEL"
