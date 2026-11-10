from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='deteccion_pista_pkg',
            executable='track_detection_node',
            name='track_detection_node',
            output='screen'
        ),

    ])