#!/bin/bash

# Obtener el directorio del script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_env.sh"

echo "  W - Avanzar"
echo "  A - Girar izquierda"
echo "  D - Girar derecha"
echo "  X - Retroceder"
echo "  S - Detener"
echo "  Q - Aumentar velocidad lineal"
echo "  Z - Disminuir velocidad lineal"
echo "  E - Aumentar velocidad angular"
echo "  C - Disminuir velocidad angular"
echo "  ESPACIO - Forzar detención"

# Lanzar teleop
ros2 run turtlebot3_teleop teleop_keyboard