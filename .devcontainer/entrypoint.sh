#!/bin/bash

#
# ROS environment setup
#

# Source the ROS2 environment.
source /opt/ros/$ROS_DISTRO/setup.bash

CRL_WS=/opt/crl
ROS2_WS=$CRL_WS/ros_ws

cd $ROS2_WS

#
# Fix file ownership issues
#

# Fix ownership of only the directories created by thhe image build process.
#
# Dev Containers remap the container user to the host UID/GID at runtime.
# However, directories created in the image were chown'd to the build-time
# ubuntu user/group (1000:1000).  Here we only chown the directories created by
# the image build process so that all mounted files/directories maintain their
# existing permissions.
sudo chown "$(id -u):$(id -g)" $CRL_WS $ROS2_WS $ROS2_WS/src

#
# Install any additional ROS2 dependencies.
#

echo
echo "Installing additional ROS2 dependencies..."
sudo apt update

echo
echo "Updating ROS2 dependencies..."
rosdep update

echo
echo "Installing ROS2 dependencies..."
sudo rosdep install --from-paths $ROS2_WS/src --rosdistro $ROS_DISTRO -y --ignore-src

#
# Build and source the ROS2 workspace.
#

echo
echo "Building ROS2 workspace..."

# Build with compile commands for better IntelliSense support.
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Create symlink for clangd to find compile_commands.json
if [ -f "$ROS2_WS/src/multisense_ros2/symlink_compile_commands.sh" ]; then
    "$ROS2_WS/src/multisense_ros2/symlink_compile_commands.sh"
fi

source $ROS2_WS/install/setup.bash

#
# Set up shell initialization commands to run on every new shell
#

if [ -f ~/.bashrc ]; then
    # Add a marker to identify our custom section
    if ! grep -q "# multisense_ros2 DevContainer Shell Init" ~/.bashrc; then
        cat >> ~/.bashrc << 'EOF'

# multisense_ros2 DevContainer Shell Init
# Source ROS2 environment
if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
fi

# Source workspace if present
if [ -f $ROS2_WS/install/setup.bash ]; then
    source $ROS2_WS/install/setup.bash
fi

# Change to workspace directory
cd $ROS2_WS

EOF
    fi
fi

#
# Execute the command passed to the entrypoint.
#

exec "$@"
