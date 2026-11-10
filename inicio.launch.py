
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package="deteccion_pista",
            executable="nodo_lectura_rs",
            name="camera_node",
            output="screen"
        ),

        Node(
            package="deteccion_pista",
            executable="lane_detection_node",
            name="lane_detection_node",
            output="screen"
        ),

        # Node(
        #     package="deteccion_pista",
        #     executable="obstacle_detection_node",
        #     name="obstacle_detection_node",
        #     output="screen"
        # )

        # Node(
        #     package="nodo_principal",
        #     executable="main_node",
        #     name="main_node",
        #     output="screen"
        # ),

        # Node(
        #     package="arduino_interface",
        #     executable="arduino_node",
        #     name="arduino_node",
        #     output="screen"
        # ),

        Node(
            package="deteccion_persona",
            executable="face_detection_ROS",
            name="face_detection_node",
            output="screen"
        # ),

        # Node(
        #     package="modulo_gps",
        #     executable="gps_ROS",
        #     name="gps_node",
        #     output="screen"
        )
    ])