#!/bin/bash

# Obtener el directorio del script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_env.sh"

# Ejecutar el SLAM toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
