#!/bin/bash

# Compilar primero
colcon build --packages-select busqueda_tesoro minimal_interfaces tesoro_pkg

# Terminal 1: Lanzar la simulación de Gazebo con el mundo de la casa
x-terminal-emulator -T "Gazebo - Tarea 2" -e bash -c "
echo '=== GAZEBO - Simulación ===';
echo 'Lanzando Gazebo con el mundo de la casa...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
source ../turtlebot3_ws/install/setup.bash;
export TURTLEBOT3_MODEL=waffle;
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py headless:=False;
exec bash" &

# Terminal 2: Lanzar Nav2
x-terminal-emulator -T "Nav2 - Navegacion" -e bash -c "
echo '=== NAV2 - Sistema de Navegación ===';
echo 'Presiona ENTER para lanzar Nav2...';
read;
echo 'Lanzando Nav2...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
export TURTLEBOT3_MODEL=waffle;
ros2 launch nav2_bringup bringup_launch.py map:=mapas/casa_map.yaml use_sim_time:=True params_file:=src/busqueda_tesoro/config/turtlebot3_params.yaml;
exec bash" &

# Terminal 3: Lanzar RViz
x-terminal-emulator -T "RViz - Visualizacion" -e bash -c "
echo '=== RVIZ - Visualización ===';
echo 'Presiona ENTER para lanzar RViz...';
read;
echo 'Lanzando RViz...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
export TURTLEBOT3_MODEL=waffle;
ros2 launch nav2_bringup rviz_launch.py;
exec bash" &

# Terminal 4: Lanzar el nodo del tesoro
x-terminal-emulator -T "Tesoro - Publicador" -e bash -c "
echo '=== NODO DEL TESORO ===';
echo 'Presiona ENTER para iniciar el nodo del tesoro...';
read;
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
echo 'Iniciando nodo del tesoro...';
echo '';
ros2 run tesoro_pkg tesoro_nodo;
exec bash" &

# Terminal 5: Lanzar el nodo de búsqueda
x-terminal-emulator -T "Busqueda Autonoma - Tarea 2" -e bash -c "
echo '=== BÚSQUEDA AUTÓNOMA DEL TESORO - TAREA 2 ===';
echo 'Presiona ENTER para iniciar la búsqueda autónoma...';
read;
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
echo 'Iniciando nodo de búsqueda autónoma...';
ros2 run busqueda_tesoro busqueda_nodo --ros-args --param use_sim_time:=true;
exec bash" &