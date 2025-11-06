#!/bin/bash

# Obtener el directorio del script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_env.sh"

# Ruta relativa al mapa
MAPA_PATH="$SCRIPT_DIR/../mapas/casa_map.yaml"

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:="$MAPA_PATH"