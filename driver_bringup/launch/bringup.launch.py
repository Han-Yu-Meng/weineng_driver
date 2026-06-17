import os
import subprocess
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Check and set Cyclone DDS for humble
    ros_distro = os.getenv('ROS_DISTRO', '')
    if ros_distro == 'humble':
        result = subprocess.run(
            ['dpkg', '-s', 'ros-humble-rmw-cyclonedds-cpp'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        if result.returncode == 0:
            print("Cyclone DDS is already installed.")
        else:
            print("Cyclone DDS not installed. Installing now...")
            subprocess.run(['sudo', 'apt-get', 'update'], check=True)
            subprocess.run(
                ['sudo', 'apt-get', 'install', '-y', 'ros-humble-rmw-cyclonedds-cpp'],
                check=True
            )

    # Get config path
    bringup_share = get_package_share_directory('driver_bringup')
    config_path = os.path.join(bringup_share, 'config', 'airy_config.yaml')

    declare_config_path = DeclareLaunchArgument(
        'config_path',
        default_value=config_path,
        description='Path to the LiDAR config YAML file'
    )

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Fines robot communication'
    )

    # LiDAR driver node
    rslidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        namespace='',
        output='screen',
        parameters=[{'config_path': LaunchConfiguration('config_path')}],
    )

    # Fines serial communication node
    fines_serial_node = Node(
        package='fines_serial',
        executable='fines_serial',
        name='serial_station',
        namespace='',
        output='screen',
        arguments=[LaunchConfiguration('serial_port')],
    )

    ld = LaunchDescription()
    ld.add_action(declare_config_path)
    ld.add_action(declare_serial_port)

    if ros_distro == 'humble':
        ld.add_action(
            SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
        )

    ld.add_action(rslidar_node)
    ld.add_action(fines_serial_node)

    return ld
