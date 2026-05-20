#!/bin/bash

#
# Create a symlink from ros_ws/compile_commands.json to ros_ws/build/compile_commands.json so clangd can find it when searching upward from source files.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="$(dirname "$(dirname "${SCRIPT_DIR}")")"

BUILD_DB="${ROS2_WS}/build/compile_commands.json"
LINK_PATH="${ROS2_WS}/compile_commands.json"

# Remove existing symlink or file if it exists
if [ -L "${LINK_PATH}" ] || [ -f "${LINK_PATH}" ]; then
	rm -f "${LINK_PATH}"
fi

# Create symlink if build database exists
if [ -f "${BUILD_DB}" ]; then
	ln -s "build/compile_commands.json" "${LINK_PATH}"
	echo "Created symlink: ${LINK_PATH} -> build/compile_commands.json"
else
	echo "Build compile_commands.json not found, skipping symlink creation"
fi
