import rclpy
from rclpy.node import Node
from minimal_interfaces.srv import ComandoNavegacion
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time


class ServidorComandos(Node):
    def __init__(self):
        super().__init__('servidor_comandos')
        # Crear el servidor ROS2 para recibir los comandos
        self.srv = self.create_service(
            ComandoNavegacion, 
            'comando_navegacion', 
            self.comando_callback
        )
        # Inicializa el navigator Nav2 para control del robot
        self.navigator = BasicNavigator()
        self.get_logger().info('Servidor de comandos iniciado')

    def crear_pose(self, x, y, theta):
        # Funcion auxiliar para crear poses con coordenadas del mapa para navegación
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = theta
        pose.pose.orientation.w = 1.0
        return pose

    def comando_callback(self, request, response):
        # Callback principal que procesa los comandos recibidos
        self.get_logger().info(f'Comando recibido: {request.comando}')

        # Arbol de decisiones para enrutar los comandos a funciones específicas
        if request.comando == "Patrullar":
            response.exito, response.mensaje = self.patrullar()
        elif request.comando == "GoToExit":
            response.exito, response.mensaje = self.ir_a_salida()
        else:
            response.exito = False
            response.mensaje = f"Comando '{request.comando}' no reconocido."
        
        return response

    def patrullar(self):
        # La función principal para hacer el patrullaje continuo por todos los puntos de la casa
        self.get_logger().info('Iniciando patrullaje continuo...')

        # Este array define la ruta de patrullaje cubriendo toda la casa desde abajo hacia arriba
        puntos_patrullaje = [
            self.crear_pose(2.0, -2.0, 0.0),    # salida
            self.crear_pose(-6.0, -3.0, 0.0),   # habitacion abajo
            self.crear_pose(-6.0, 4.0, 0.0),    # habitacion abajo izquierda
            self.crear_pose(-1.0, 4.0, 0.0),    # habitacion grande
            self.crear_pose(7.0, 4.0, 0.0),     # habitacion arriba
            self.crear_pose(1.0, 2.0, 0.0),     # habitacion en medio
            self.crear_pose(7.0, 4.0, 0.0),     # habitacion arriba
            self.crear_pose(6.0, -1.0, 0.0)     # habitacion mesa
        ]
        
        ronda = 0
        # El robot patrullará continuamente
        while True:
            ronda += 1
            self.get_logger().info(f'Iniciando ronda de patrullaje {ronda}')
            
            for i, punto in enumerate(puntos_patrullaje):
                self.get_logger().info(f'Ronda {ronda}: Navegando al punto {i+1}/{len(puntos_patrullaje)}')
                # Enviar objetivo de navegación a Nav2
                self.navigator.goToPose(punto)

                # Esperamos a que Nav2 complete la navegación
                while not self.navigator.isTaskComplete():
                    time.sleep(0.1)
                
                resultado = self.navigator.getResult()
                if resultado != TaskResult.SUCCEEDED:
                    # Si no podemos llegar a un punto por cualquier motivo continuamos con el siguiente porque este puede desatascar el robot
                    self.get_logger().warning(f"Fallo en punto {i+1}, continuando patrullaje...")
                    continue
            
            self.get_logger().info(f'Ronda {ronda} completada, iniciando siguiente ronda...')
            time.sleep(1.0)  # Por si acaso, pequeña pausa entre rondas

    def ir_a_salida(self):
        # Esta funcion permite navegar a la posición de salida. Usa la misma estructura que el patrullaje
        self.get_logger().info('Navegando a la salida...')
        
        # Coordenadas de la salida
        salida = self.crear_pose(2.0, -2.0, 0.0)
        
        # Envia el comando de navegación a Nav2
        self.navigator.goToPose(salida)
        
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)
        
        resultado = self.navigator.getResult()
        
        # Devolver el éxito/fallo al cliente
        if resultado == TaskResult.SUCCEEDED:
            mensaje = "Salida alcanzada con éxito"
            self.get_logger().info(mensaje)
            return True, mensaje
        else:
            mensaje = "No se pudo alcanzar la salida"
            self.get_logger().error(mensaje)
            return False, mensaje


def main(args=None):
    rclpy.init(args=args)
    servidor = ServidorComandos()
    
    try:
        rclpy.spin(servidor)
    except KeyboardInterrupt:
        pass
    finally:
        servidor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
