#!/usr/bin/env bash
set -euo pipefail

# Example location.
LAT="${1:-28.6139}"
LON="${2:-77.2090}"
RADIUS="${3:-500}"

source /opt/ros/jazzy/setup.bash
source ~/drone_ws/.venv/bin/activate
source ~/drone_ws/install/setup.bash

ros2 run drone_map_manager generate   --lat "$LAT"   --lon "$LON"   --radius "$RADIUS"
