from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='gps_pkg',
            executable='gps_node',
            name='gps_node',
            output='screen'
        ),

    ])