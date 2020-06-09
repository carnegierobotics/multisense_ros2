# MultiSense ROS2 Driver
A port of the MultiSense ROS1 driver to ROS2

This driver was tested using Eloquent Elusor. Earlier ROS2 distros are not supported

## Notable Changes
- Replace dyanmic reconfigure interface with ROS2 parameters
- Remove support for sensor firmware versions < 3.3
- Remove support for monocular and BCAM configurations
- Move custom messages to standalone multisense_msgs package
- General cleanup and port to c++17

## Build

```
source /opt/ros/<ros2_distro>/setup.bash
mkdir ros2_ws && cd ros2_ws
git clone https://github.com/carnegierobotics/multisense_ros2 src
colcon build
```

## Run

`ros2 run multisense_ros ros_driver`
