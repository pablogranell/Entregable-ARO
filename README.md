# Trabajo ROS2 - TurtleBot3

## TAREA 0: Mapeo de la Casa

**IMPORTANTE:** Todos los comandos se ejecutan desde el directorio `turtlebot3_ws/src`

### Proceso de Mapeo (4 terminales)

**Terminal 1 - Gazebo:**
```bash
bash scripts/paso1_gazebo.sh
```

**Terminal 2 - SLAM:**
```bash
bash scripts/paso2_slam.sh
```

**Terminal 3 - Teleop (controla con W/A/S/D/X):**
```bash
bash scripts/paso3_teleop.sh
```

**Terminal 4 - Guardar mapa:**
```bash
bash scripts/paso4_mapa.sh
```

### Verificar con Nav2
Cierra SLAM y Teleop (Ctrl+C), mantén Gazebo corriendo:
```bash
bash scripts/paso5_nav2.sh
```

## Instalación
```bash
cd ~/turtlebot3_ws/src/
git clone https://github.com/pablogranell/Trabajo-ARO .
```

```bash
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
sudo dpkg --install ros-humble-tesoro-pkg_0.0.0-0jammy_amd64.deb
```

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```
