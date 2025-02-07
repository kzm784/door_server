import os
import launch
import launch_ros

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    door_server_config_arg = DeclareLaunchArgument(
        'door_server_config',
        default_value=os.path.join(
            get_package_share_directory('door_server'),
            'config',
            'config_door_server.yaml'
        ),
        description='Path to door_server config file'
    )
    ld.add_action(door_server_config_arg)

    door_server_config = LaunchConfiguration('door_server_config')
    door_server_node = Node(
        package='door_server',
        executable='door_server_node',
        name='door_server_node',
        output='screen',
        parameters=[door_server_config]
    )
    ld.add_action(door_server_node)


    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('door_server'),
                'launch',
                'realsense_d435i.launch.py'
            )
        )
    )
    ld.add_action(rs_launch)

    return ld
