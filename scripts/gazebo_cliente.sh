#!/bin/bash

# Obtener el directorio del script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_env.sh"

# Ejecutar la interfaz de gazebo
gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so