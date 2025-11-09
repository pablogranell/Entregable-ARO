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

## TAREA 1: Navegación Autónoma y Servicio de Comandos

### Ejecutar todo (5 terminales)
Desde el directorio raiz Entregable-ARO:
```bash
bash scripts/ejecutar_todo.sh
```

NOTA: Pulsar ENTER en cada terminal para continuar el proceso una vez nos aseguramos de que el servidor de gazebo esta funcionando correctamente.

Cuando todo haya cargado, seleccionar la posicion inicial aproximada del robot con 2D pose estimate en RViz.

Luego ejecutar en otra terminal:
```bash
source src/install/setup.bash
ros2 run servicio_comandos cliente_comandos Patrullar
ros2 run servicio_comandos cliente_comandos GoToExit
```

El robot empezara a navegar en base a los comandos recibidos.

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
source src/install/setup.bash
```
