#!/bin/bash

# Obtener el directorio del script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_env.sh"

# Crear directorio de mapas relativo al script
MAPAS_DIR="$SCRIPT_DIR/../mapas"
mkdir -p "$MAPAS_DIR"
cd "$MAPAS_DIR"

# Guardar el mapa usando map_saver_cli
ros2 run nav2_map_server map_saver_cli -f casa_map