from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        ld = LaunchDescription()
        
        config = os.path.join(
            get_package_share_directory('trect2'),
            'config',
            'trect_params.yaml'
            )

        trect2_node = Node(
            package='trect2',
            executable='trect2',
            name='trect2',
            parameters=[config],
            output='screen'
        )

        turtlesim_node = Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        )

        ld.add_action(trect2_node)
        ld.add_action(turtlesim_node)

        return ld