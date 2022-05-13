# MultiSense ROS2 Driver

A port of the MultiSense ROS1 driver to ROS2

This driver was tested using Foxy Fitzroy. Earlier ROS2 distros are not supported

## Notable Changes

- Replace dynamic reconfigure interface with ROS2 parameters
- Remove support for sensor firmware versions < 3.3
- Remove support for monocular and BCAM configurations
- Move custom messages to standalone multisense_msgs package
- Merge multisense_bringup and multisense_description into the multisense_ros package
- General cleanup and port to c++17 and gcc 9.3.0
- Update launch file to new ROS2 launch file format
- multisense_lib has LibMultiSense as a submodule

## Build

```
source /opt/ros/<ros2_distro>/setup.bash
mkdir ros2_ws && cd ros2_ws
git clone --recurse-submodules https://github.com/carnegierobotics/multisense_ros2 src
colcon build
source install/setup.bash
```

The launch file depends on xacro being installed. To install xacro execute the following command.
```
sudo apt install ros-<distro>-xacro
```

If your release does not have a pre-built xacro package, you can build it manually
alongside the multisense_ros2 driver. 

Execute the following commands to build xacro from source. This assmes the workspace setup and clone instructions above were followed.

```
cd ros2_ws/src
git clone -b dashing-devel https://github.com/ros/xacro.git
cd ..
colcon build
source install/setup.bash
```

## Launch

`ros2 launch multisense_ros multisense_launch.py`

For the full set of launch arguments use

`ros2 launch multisense_ros multisense_launch.py -s`
