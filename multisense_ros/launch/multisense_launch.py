#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix


def multisense_setup(context, *args, **kwargs):
    # Resolve LaunchConfiguration values that we need as Python strings
    ip = LaunchConfiguration('ip_address').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)

    # Keep these as substitutions; ParameterValue will coerce them
    mtu = LaunchConfiguration('mtu')
    reconnect = LaunchConfiguration('reconnect')
    reconnect_timeout_s = LaunchConfiguration('reconnect_timeout_s')

    # Params file: normalize and fail loudly if user provided a bad path
    params_file = LaunchConfiguration('params_file').perform(context).strip()
    if params_file:
        params_file = os.path.abspath(os.path.expanduser(params_file))
        if not os.path.isfile(params_file):
            raise RuntimeError( # Use RuntimeError for portability across distros
                f"params_file was provided but does not exist: '{params_file}'. "
                "Pass a valid path, e.g. params_file:=/absolute/path/to/params.yaml"
            )
        print(f"[multisense_launch] Using params_file: {params_file}")

    # Inline parameters (these are always applied)
    params = {
        'sensor_ip': ip,
        'sensor_mtu': ParameterValue(mtu, value_type=int),
        'tf_prefix': namespace,
        'reconnect': ParameterValue(reconnect, value_type=bool),
        'reconnect_timeout_s': ParameterValue(reconnect_timeout_s, value_type=int),
    }

    node_args = {
        'package': 'multisense_ros',
        'executable': 'ros_driver',
        'output': 'screen',
        'namespace': namespace,
    }

    # Apply YAML first, then inline params (inline can override YAML on key conflicts)
    node_args['parameters'] = ([params_file, params] if params_file else [params])

    return [Node(**node_args)]

def generate_launch_description():
    sensor = DeclareLaunchArgument(
        name='sensor',
        default_value='S21',
        description='Type of multisense: S21, SL, S7, S7S, S27, S30, KS21, KS21i'
    )

    namespace = DeclareLaunchArgument(
        name='namespace',
        default_value='multisense',
        description='Namespace for this MultiSense instance'
    )

    mtu = DeclareLaunchArgument(
        name='mtu',
        default_value='1500',
        description='Sensor MTU'
    )

    ip_address = DeclareLaunchArgument(
        name='ip_address',
        default_value='10.66.171.21',
        description='Sensor IP address'
    )

    launch_robot_state_publisher = DeclareLaunchArgument(
        name='launch_robot_state_publisher',
        default_value='true',  # use lowercase for launch boolean parsing
        description='Launch the robot_state_publisher'
    )

    reconnect = DeclareLaunchArgument(
        name='reconnect',
        default_value='false',  # use lowercase for launch boolean parsing
        description='Reconnect on camera issues'
    )

    reconnect_timeout_s = DeclareLaunchArgument(
        name='reconnect_timeout_s',
        default_value='5',
        description='Timeout in seconds before the driver configures a reconnect'
    )

    # Declare params_file ONCE (no duplicates)
    params_file_arg = DeclareLaunchArgument(
        name='params_file',
        default_value='',
        description='Optional path to a YAML parameter file'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=[LaunchConfiguration('namespace')],
        condition=IfCondition(LaunchConfiguration('launch_robot_state_publisher')),
        parameters=[{
            'robot_description': Command([
                PathJoinSubstitution([FindPackagePrefix('xacro'), 'bin', 'xacro']),  # no trailing space
                ' ',
                PathJoinSubstitution([
                    get_package_share_directory('multisense_ros'),
                    'urdf',
                    LaunchConfiguration('sensor'),
                    'standalone.urdf.xacro'
                ]),
                ' ',
                'name:=', LaunchConfiguration('namespace'),
            ])
        }]
    )

    return LaunchDescription([
        sensor,
        namespace,
        mtu,
        ip_address,
        launch_robot_state_publisher,
        reconnect,
        reconnect_timeout_s,
        params_file_arg,
        robot_state_publisher,
        OpaqueFunction(function=multisense_setup),
    ])
