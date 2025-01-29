import os
import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # Set the path to the door_server config
    waypoint_navigator_config = launch.substitutions.LaunchConfiguration(
        'door_server_config',
        default=os.path.join(
            get_package_share_directory('door_server'),
                'config',
                'config_door_server.yaml'
        )
    )

    waypoint_navigator_node = Node(
        package='door_server',
        executable='door_server_node',
        name='door_server_node',
        output='screen',
        parameters=[waypoint_navigator_config]
    )
    
    ld.add_action(waypoint_navigator_node)

    return ld