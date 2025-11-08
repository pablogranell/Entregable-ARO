import rclpy
from rclpy.node import Node
from minimal_interfaces.srv import ComandoNavegacion
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time


class ServidorComandos(Node):
    def __init__(self):
        super().__init__('servidor_comandos')
        self.srv = self.create_service(
            ComandoNavegacion, 
            'comando_navegacion', 
            self.comando_callback
        )
        self.navigator = BasicNavigator()
        self.get_logger().info('Servidor de comandos iniciado')

    def crear_pose(self, x, y, theta):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Conversión de theta a quaternion
        pose.pose.orientation.z = theta
        pose.pose.orientation.w = 1.0
        
        return pose

    def comando_callback(self, request, response):
        self.get_logger().info(f'Comando recibido: {request.comando}')
        
        if request.comando == "Patrullar":
            response.exito, response.mensaje = self.patrullar()
        elif request.comando == "GoToExit":
            response.exito, response.mensaje = self.ir_a_salida()
        else:
            response.exito = False
            response.mensaje = f"Comando '{request.comando}' no reconocido."
        
        return response

    def patrullar(self):
        self.get_logger().info('Iniciando patrullaje...')
        
        # Puntos de patrullaje e intermedios para recorrer toda la casa
        puntos_patrullaje = [
            self.crear_pose(2.0, -2.0, 0.0),    # salida
            self.crear_pose(-6.0, -3.0, 0.0),   # habitacion abajo
            self.crear_pose(-6.0, 4.0, 0.0),    # habitacion abajo izquierda
            self.crear_pose(-1.0, 4.0, 0.0),    # habitacion grande
            self.crear_pose(7.0, 4.0, 0.0),     # habitacion arriba
            self.crear_pose(1.0, 2.0, 0.0),     # habitacion en medio
            self.crear_pose(7.0, 4.0, 0.0),     # habitacion arriba
            self.crear_pose(6.0, -2.0, 0.0)     # habitacion mesa
        ]
        
        for i, punto in enumerate(puntos_patrullaje):
            self.get_logger().info(f'Navegando al punto {i+1}/{len(puntos_patrullaje)}')
            self.navigator.goToPose(punto)
            
            while not self.navigator.isTaskComplete():
                time.sleep(0.1)
            
            resultado = self.navigator.getResult()
            if resultado != TaskResult.SUCCEEDED:
                mensaje = f"Patrullaje fallido en punto {i+1}"
                self.get_logger().error(mensaje)
                return False, mensaje
        
        mensaje = "Patrullaje completado con éxito"
        self.get_logger().info(mensaje)
        return True, mensaje

    def ir_a_salida(self):
        self.get_logger().info('Navegando a la salida...')
        
        # Coordenadas de la salida
        salida = self.crear_pose(2.0, -2.0, 0.0)
        
        self.navigator.goToPose(salida)
        
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)
        
        resultado = self.navigator.getResult()
        
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
