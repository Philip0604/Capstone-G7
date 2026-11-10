from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='deteccion_persona_pkg',
            executable='face_detection_node',
            name='face_detection_node',
            output='screen'
        ),

    ])