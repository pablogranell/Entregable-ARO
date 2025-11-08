# Servicio de Comandos de Navegación

Este paquete implementa un servicio de comandos para controlar la navegación del Turtlebot3 en ROS2.

## Estructura del Proyecto

- **minimal_interfaces**: Contiene la interfaz de servicio personalizada `ComandoNavegacion.srv`
- **servicio_comandos**: Contiene el servidor y cliente de servicios

## Interfaz de Servicio

La interfaz `ComandoNavegacion.srv` define:

**Solicitud:**
- `string comando`: El comando a ejecutar ("Patrullar" o "GoToExit")

**Respuesta:**
- `bool exito`: Indica si el comando se ejecutó correctamente
- `string mensaje`: Mensaje descriptivo del resultado

## Comandos Disponibles

### 1. Patrullar
Hace que el Turtlebot3 navegue por todas las estancias de la casa, visitando puntos intermedios optimizados.

### 2. GoToExit
Navega el robot hacia la salida de la casa.

## Compilación

Desde el directorio del workspace:

```bash
cd ~/ARO/Entregable-ARO
colcon build --packages-select minimal_interfaces servicio_comandos
source install/setup.bash
```

## Uso

### 1. Iniciar el servidor

En una terminal:

```bash
ros2 run servicio_comandos servidor_comandos
```

### 2. Usar el cliente

En otra terminal:

**Para patrullar:**
```bash
ros2 run servicio_comandos cliente_comandos Patrullar
```

**Para ir a la salida:**
```bash
ros2 run servicio_comandos cliente_comandos GoToExit
```

## Prerrequisitos

- ROS2 Humble
- Nav2
- Turtlebot3 simulación
- Un mapa cargado y Nav2 ejecutándose

## Notas

- Los puntos de patrullaje y la ubicación de la salida deben ajustarse según el mapa específico utilizado
- El servidor requiere que Nav2 esté activo y funcionando
- El robot debe estar localizado en el mapa antes de enviar comandos
