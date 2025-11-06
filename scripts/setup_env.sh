#!/bin/bash

# Detectar la ubicación del workspace automáticamente
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
SRC_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace
if [ -f "$WORKSPACE_DIR/turtlebot3_ws/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/turtlebot3_ws/install/setup.bash"
else
    echo "Advertencia: No se encontró $WORKSPACE_DIR/turtlebot3_ws/install/setup.bash"
    echo "Ejecuta 'colcon build' en el workspace primero"
fi

# Configurar modelo TurtleBot3
export TURTLEBOT3_MODEL=burger

# Configurar Gazebo
GAZEBO_MODELS="$SRC_DIR/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models"
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$GAZEBO_MODELS

echo "   - ROS_DISTRO: $ROS_DISTRO"
echo "   - TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo "   - Workspace: $WORKSPACE_DIR"
