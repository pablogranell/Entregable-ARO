#!/bin/bash

# Obtener el directorio del script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_env.sh"

# Ejecutar la navegación con Nav2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
