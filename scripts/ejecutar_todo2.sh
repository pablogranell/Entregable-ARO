#!/bin/bash

# Compilar primero
colcon build --packages-select busqueda_tesoro minimal_interfaces

# Terminal 1: Lanzar la simulación de Gazebo
x-terminal-emulator -T "1. Gazebo" -e bash -c "
echo 'Lanzando Gazebo...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
source ../turtlebot3_ws/install/setup.bash;
export TURTLEBOT3_MODEL=waffle;
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py headless:=True;
exec bash" &

# Terminal 2: Lanzar Nav2
x-terminal-emulator -T "2. Navegacion" -e bash -c "
echo 'Presiona ENTER para lanzar Nav2...';
read;
echo 'Lanzando Nav2...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
export TURTLEBOT3_MODEL=waffle;
ros2 launch nav2_bringup bringup_launch.py map:=mapas/casa_map.yaml use_sim_time:=True params_file:=src/busqueda_tesoro/config/turtlebot3_params.yaml;
exec bash" &

# Terminal 3: Lanzar RViz
x-terminal-emulator -T "3. RViz" -e bash -c "
echo 'Presiona ENTER para lanzar RViz...';
read;
echo 'Lanzando RViz...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
export TURTLEBOT3_MODEL=waffle;
ros2 launch nav2_bringup rviz_launch.py;
exec bash" &

# Terminal 4: Lanzar el nodo del tesoro
x-terminal-emulator -T "4. Publicador de Tesoro" -e bash -c "
echo 'Presiona ENTER para iniciar el publicador del tesoro...';
read;
echo 'Iniciando publicador del tesoro...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
ros2 run tesoro_pkg tesoro_nodo;
exec bash" &

# Terminal 5: Lanzar el nodo de búsqueda
x-terminal-emulator -T "5. Busqueda autonoma del tesoro" -e bash -c "
echo 'Presiona ENTER para iniciar la búsqueda autónoma...';
read;
echo 'Iniciando nodo de búsqueda autónoma...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
ros2 run busqueda_tesoro busqueda_nodo;
exec bash" &