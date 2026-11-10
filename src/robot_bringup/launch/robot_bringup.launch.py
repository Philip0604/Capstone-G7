
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

        # Adquisición de imágenes
        incluir_launch('lectura_sensores_pkg', '__init__.py'),

        # Percepción
        incluir_launch('deteccion_persona_pkg', '__init__.py'),
        incluir_launch('deteccion_pista_pkg', '__init__.py'),
        # incluir_launch('deteccion_obstaculos_pkg', '__init__.py'),

        # GPS y navegación
        incluir_launch('gps_pkg', '__init__.py'),
        # incluir_launch('navegacion_pkg', '__init__.py'),

        # Control y comunicación
        # incluir_launch('control_pkg', '__init__.py'),
        # incluir_launch('arduino_pkg', '__init__.py'),

        # Interacción auditiva
        incluir_launch('audio_pkg', '__init__.py'),

    ])