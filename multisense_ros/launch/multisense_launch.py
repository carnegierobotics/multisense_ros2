import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix

def multisense_setup(context, *args, **kwargs):

    # Resolve all LaunchConfiguration values
    ip = LaunchConfiguration('ip_address').perform(context)
    mtu = LaunchConfiguration('mtu').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    node_args = {
        'package': 'multisense_ros',
        'executable': 'ros_driver',
        'output': 'screen',
        'namespace': namespace
    }


    # Inline parameters
    params = {
        'sensor_ip': ip,
        'sensor_mtu': int(mtu),
        'tf_prefix': namespace
    }

    # Conditionally include param file if it exists
    if params_file and os.path.isfile(params_file):
        node_args['parameters'] = [params_file, params]
    else:
        node_args['parameters'] = [params]

    return [Node(**node_args)]

def generate_launch_description():

    sensor = DeclareLaunchArgument(name='sensor',
                                   default_value='S21',
                                   description='Type of multisense: S21, SL, S7, S7S, S27, S30, KS21, KS21i')

    namespace = DeclareLaunchArgument(name='namespace',
                                      default_value='multisense',
                                      description='Namespace for this MultiSense instance')

    mtu = DeclareLaunchArgument(name='mtu',
                                default_value='1500',
                                description='Sensor MTU')

    ip_address = DeclareLaunchArgument(name='ip_address',
                                       default_value='10.66.171.21',
                                       description='Sensor IP address')


    launch_robot_state_publisher = DeclareLaunchArgument(name='launch_robot_state_publisher',
                                                         default_value='True',
                                                         description='Launch the robot_state_publisher')

    params_file = DeclareLaunchArgument('params_file',
                                        default_value='',
                                        description='Path to YAML parameter config file')

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 namespace=[LaunchConfiguration('namespace')],
                                 condition=IfCondition(LaunchConfiguration('launch_robot_state_publisher')),
                                 parameters=[{'robot_description': Command([
                                             PathJoinSubstitution([FindPackagePrefix('xacro'), 'bin', 'xacro ']),
                                             PathJoinSubstitution([get_package_share_directory('multisense_ros'),
                                                                   'urdf',
                                                                   LaunchConfiguration('sensor'),
                                                                   'standalone.urdf.xacro']),
                                             " name:=", LaunchConfiguration('namespace')])}])

    return LaunchDescription([
                              DeclareLaunchArgument(
                                  'params_file',
                                  default_value='',
                                  description='Optional path to a YAML parameter file'
                              ),
                              sensor,
                              namespace,
                              mtu,
                              ip_address,
                              launch_robot_state_publisher,
                              robot_state_publisher,
                              params_file,
                              OpaqueFunction(function=multisense_setup)
                              ])
