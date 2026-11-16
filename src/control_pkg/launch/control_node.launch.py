from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='control_pkg',
            executable='pid_message_node',
            name='pid_message_node',
            output='screen'
        ),

    ])