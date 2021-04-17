from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('trect'),
        'config',
        'trect_params.yaml'
        )

    turtle_rect = Node(
        package='trect',
        executable='turtle_rect',
        name='turtle_rect',
        parameters=[config],
        output='screen'
    )

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        output='screen'
    )

    ld.add_action(turtle_rect)
    ld.add_action(turtlesim_node)

    return ld