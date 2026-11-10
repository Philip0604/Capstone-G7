from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='lectura_sensores_pkg',
            executable='realsense_node',
            name='realsense_node',
            output='screen'
        ),

        # Node(
        #     package='lectura_sensores_pkg',
        #     executable='webcam_trasera_node',
        #     name='webcam_trasera_node',
        #     output='screen'
        # ),

    ])