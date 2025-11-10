#!/bin/bash

SCRIPT_DIR=$(pwd)
cd src
echo "path: $(pwd)"
colcon build --packages-select servicio_comandos minimal_interfaces
cd $SCRIPT_DIR
echo "path: $(pwd)"

# Terminal 1: Lanzar la simulación de Gazebo con el mundo de la casa
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
echo 'Lanzando Navegación...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
export TURTLEBOT3_MODEL=waffle;
ros2 launch nav2_bringup bringup_launch.py map:=mapas/casa_map.yaml use_sim_time:=True params_file:=src/busqueda_tesoro/config/turtlebot3_params.yaml
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

# Terminal 4: Lanzar el servidor de comandos
x-terminal-emulator -T "4. Servidor de Comandos" -e bash -c "
echo 'Presiona ENTER para lanzar el servidor...';
read;
echo 'Lanzando Servidor de Comandos...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
ros2 run servicio_comandos servidor_comandos;
exec bash" &

# Terminal 5: Lanzar el cliente de comandos (con instrucciones)
x-terminal-emulator -T "5. Comando Patrullar" -e bash -c "
echo 'Presiona ENTER para lanzar el comando...';
read;
echo 'Lanzando comando...';
source /opt/ros/humble/setup.bash;
source src/install/setup.bash;
ros2 run servicio_comandos cliente_comandos Patrullar
exec bash" &