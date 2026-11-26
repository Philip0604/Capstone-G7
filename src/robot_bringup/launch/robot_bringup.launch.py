
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

def incluir_launch(paquete, archivo):
    ruta = os.path.join(
        get_package_share_directory(paquete),
        'launch',
        archivo
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ruta)
    )


def generate_launch_description():

    return LaunchDescription([
        # Agente micro-ROS: puente Raspberry <-> Arduino GIGA por USB.
        # Busca solo el puerto del Arduino (no hay que escribir el nombre).
        # respawn: si el Arduino no esta conectado o se desconecta, el agente
        # termina y se vuelve a lanzar cada 3 s hasta que lo encuentre.
        ExecuteProcess(
            cmd=['ros2 run micro_ros_agent micro_ros_agent serial '
                 '--dev $(ls /dev/serial/by-id/usb-Arduino* | head -n1) -b 115200'],
            shell=True,
            output='screen',
            respawn=True,
            respawn_delay=3.0,
        ),

        # Adquisición de imágenes
        incluir_launch('lectura_sensores_pkg', 'lectura_sensores.launch.py'),

        # Percepción
        incluir_launch('deteccion_persona_pkg', 'deteccion_persona.launch.py'),
        incluir_launch('deteccion_pista_pkg', 'deteccion_pista.launch.py'),
        incluir_launch('deteccion_obstaculos_pkg', 'deteccion_obstaculo.launch.py'),

        # GPS y navegación
        incluir_launch('gps_pkg', 'gps.launch.py'),
        # incluir_launch('navegacion_pkg', 'navegacion.launch.py'),

        # Control y comunicación
        incluir_launch('control_pkg', 'control_node.launch.py'),
        # incluir_launch('arduino_pkg', 'arduino.launch.py'),

        # Interacción auditiva
        incluir_launch('audio_pkg', 'audio.launch.py'),

    ])