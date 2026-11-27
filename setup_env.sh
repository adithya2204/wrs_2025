#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Activate venv
source "$SCRIPT_DIR/.venv/bin/activate"

# ROS2 and workspace
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# Inject venv packages so ROS2 entry points can find them
export PYTHONPATH="$SCRIPT_DIR/.venv/lib/python3.10/site-packages:$PYTHONPATH"

# Workspace root, used by aizu_spider_description/launch/sim.launch.py to find the project file
export WRS_WS="$SCRIPT_DIR"

echo "Environment ready: $SCRIPT_DIR"
