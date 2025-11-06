"""
ROS 2 node to spawn a mobile robot inside a warehouse.

Author:
  - Addison Sears-Collins
  - https://automaticaddison.com
"""

import os # Biblioteca del sistema operativo
import sys # Biblioteca del entorno de tiempo de ejecución de Python
import rclpy # Biblioteca del Cliente de ROS para Python

# Biblioteca de gestión de paquetes
from ament_index_python.packages import get_package_share_directory 

# Servicio de Gazebo para spawnear un robot
from gazebo_msgs.srv import SpawnEntity

def main():

    """ Función principal para spawnear un nodo de robot """
    # Obtener argumentos de entrada del usuario
    argv = sys.argv[1:]

    # Iniciar nodo
    rclpy.init()

    # Obtener la ruta del archivo para el modelo del robot
    sdf_file_path = os.path.join(
        get_package_share_directory("warehouse_robot_spawner_pkg"), "models",
        "mobile_warehouse_robot", "model.sdf")
        
    # Crear el nodo
    node = rclpy.create_node("entity_spawner")

    # Mostrar progreso en la ventana del terminal
    node.get_logger().info(
        'Creando cliente de servicio para conectarse a `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    # Obtener el servicio spawn_entity
    node.get_logger().info("Conectando al servicio `/spawn_entity`...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...¡conectado!")

    # Obtener ruta al robot
    sdf_file_path = os.path.join(
        get_package_share_directory("warehouse_robot_spawner_pkg"), "models",
        "mobile_warehouse_robot", "model.sdf")

    # Mostrar ruta del archivo
    print(f"robot_sdf={sdf_file_path}")
    
    # Establecer datos para solicitud
    request = SpawnEntity.Request()
    request.name = argv[0]
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = argv[1]
    request.initial_pose.position.x = float(argv[2])
    request.initial_pose.position.y = float(argv[3])
    request.initial_pose.position.z = float(argv[4])

    node.get_logger().info("Enviando solicitud de servicio a `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("¡Hecho! Cerrando el nodo.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()