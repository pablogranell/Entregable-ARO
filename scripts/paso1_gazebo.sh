#!/bin/bash

# Obtener el directorio del script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_env.sh"

# Ejecutar la simulación de Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py headless:=True