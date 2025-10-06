# MultiSense ROS2 Driver

The offically supported MultiSense ROS2 driver

This driver was developed and tested using Jazzy Jalisco. Earlier ROS2 distros are not supported

## Build

Clone the MultiSense ROS2 driver

```
source /opt/ros/<ros2_distro>/setup.bash
mkdir ros2_ws && cd ros2_ws
git clone --recurse-submodules https://github.com/carnegierobotics/multisense_ros2 src
```

Ensure all the MultiSense ROS2 dependencies are installed using rosdep

```
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

Build and install the ROS2 driver

```
colcon build
source install/setup.bash
```

## Launch

`ros2 launch multisense_ros multisense_launch.py`

For the full set of launch arguments use

`ros2 launch multisense_ros multisense_launch.py -s`

## Configuration

MultiSense operation parameters including resolution, frame rate, gain, exposure gamma, etc. can be dynamically changed
at runtime via the [ROS2 parameter server](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html).

### GUI

The rqt_reconfigure GUI can be used to dynamically change camera parameters during camera operation

The following command launches the rqt_reconfigure GUI

```
ros2 run rqt_reconfigure rqt_reconfigure
```

![rqt_reconfigure_panel](./images/rqt_reconfigure.png)

NOTE: You may need to click "Refresh" on the bottom left of the rqt_reconfigure panel to see the MultiSense
configuration settings

### Command Line

The [ROS2 parameter server](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
has a command line interface to dynamically set MultiSense camera parameters at runtime.

The following command gets a full list of ROS2 parameters which can reconfigure the MultiSense at runtime

```
ros2 param list
```

#### Examples

##### Change Framerate

To get the current framerate execute the following command

```
ros2 param get /multisense/camera fps
```

To set the framerate to new value (in this example 15fps) execute the following command

```
ros2 param set /multisense/camera fps 15
```

##### Change Resolution

To get the current operating resolution execute the following command

```
ros2 param get /multisense/camera sensor_resolution
```

To set the sensor resolution to new value (in this example 1/4 resolution with 256 disparities) execute the following command

```
ros2 param set /multisense/camera sensor_resolution "[960, 600, 256]"
```

##### Disable Auto-Exposure

To get the current auto exposure execute the following command

```
ros2 param get /multisense/camera auto_exposure
```

To enable/disable the camera's auto exposure algorithm (in this example disable) execute the following command

```
ros2 param set /multisense/camera auto_exposure false
```
