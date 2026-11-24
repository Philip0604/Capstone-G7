#!/bin/bash
# Arranca todo el robot. Lo usa el servicio robot.service al encender la Raspberry,
# pero tambien se puede correr a mano: ./scripts/iniciar_robot.sh

# Ajustar si las carpetas estan en otro lugar
PROYECTO="$HOME/Proyecto/Capstone-G7"
MICROROS="$HOME/Proyecto/microros_ws"

source /opt/ros/humble/setup.bash
source "$MICROROS/install/local_setup.bash"
source "$PROYECTO/install/local_setup.bash"

cd "$PROYECTO"   # para que las rutas relativas (videos, etc.) funcionen
exec ros2 launch robot_bringup robot_bringup.launch.py