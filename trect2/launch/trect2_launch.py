from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        config = os.path.join(
            get_package_share_directory('trect2'), 'config', 'trect_params.yaml'),

        Node(
            package='trect2',
            namespace='trect2',
            executable='trect2',
            name='trect2',
            parameters=[config]
        ),
        
        Node(
            package='turtlesim',
            namespace='trect2',
            executable='turtlesim_node',
            name='sim'
        )
    ])

