#!/bin/bash

# Terminal 1: Lanzar la simulación de Gazebo con el mundo de la casa
x-terminal-emulator -T "Gazebo" -e bash -c "
echo 'Lanzando Gazebo...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
source ../turtlebot3_ws/install/setup.bash;
export TURTLEBOT3_MODEL=waffle;
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py;
exec bash" &

# Terminal 2: Lanzar Nav2
x-terminal-emulator -T "Navigation" -e bash -c "
echo '=== Terminal Navegación ===';
echo 'Presiona ENTER para lanzar Nav2...';
read;
echo 'Lanzando Navegación...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
export TURTLEBOT3_MODEL=waffle;
ros2 launch nav2_bringup bringup_launch.py map:=mapas/casa_map.yaml use_sim_time:=True;
exec bash" &

# Terminal 3: Lanzar RViz
x-terminal-emulator -T "RViz" -e bash -c "
echo '=== Terminal RViz ===';
echo 'Presiona ENTER para lanzar RViz...';
read;
echo 'Lanzando RViz...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
export TURTLEBOT3_MODEL=waffle;
ros2 launch nav2_bringup rviz_launch.py;
exec bash" &

# Terminal 4: Lanzar el servidor de comandos
x-terminal-emulator -T "Servidor de Comandos" -e bash -c "
echo '=== Terminal Servidor de Comandos ===';
echo 'Presiona ENTER para lanzar el servidor...';
read;
echo 'Lanzando Servidor de Comandos...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
ros2 run servicio_comandos servidor_comandos;
exec bash" &

# Terminal 5: Lanzar el cliente de comandos (con instrucciones)
x-terminal-emulator -T "Cliente de Gazebo" -e bash -c "
echo '=== Terminal Cliente de Gazebo ===';
echo 'Presiona ENTER para lanzar el cliente...';
read;
echo 'Lanzando Cliente de Gazebo...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so
exec bash" &