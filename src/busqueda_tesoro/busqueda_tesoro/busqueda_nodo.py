import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import time

class BusquedaTesoroNode(Node):
    def __init__(self):
        super().__init__('busqueda_tesoro_node')
        
        # Publicadores para control del robot y activación de búsqueda
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.busqueda_pub = self.create_publisher(Bool, '/busquedaTesoro', 10)
        
        # Suscripción al topic que proporciona el vector de distancia al tesoro
        self.distancia_sub = self.create_subscription(
            Vector3,
            '/distanciaTesoro',
            self.distancia_callback,
            10
        )
        
        # Suscripción a odometría para conocer la posición actual del robot en el mapa
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Instanciar el Nav2 para navegar al objetivo
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Variables para almacenar las distancias recibidas del tesoro
        self.distancia_x = None
        self.distancia_y = None
        self.distancia_z = None
        self.robot_x = None
        self.robot_y = None
        
        # Parámetros de control de búsqueda
        self.tiempo_inicio = None
        self.tiempo_limite = 90  # Límite de tiempo para encontrar el tesoro
        self.distancia_objetivo = 0.5  # Distancia a la que se considera encontrado
        self.encontrado = False
        
        # Variables para gestión de Nav2
        self.goal_handle = None
        self.nav_result_future = None
        
    def distancia_callback(self, msg):
        # Actualiza el vector de distancia al tesoro cuando se recibe información
        self.distancia_x = msg.x
        self.distancia_y = msg.y
        self.distancia_z = msg.z
    
    def odom_callback(self, msg):
        # Actualiza la posición actual del robot cuando se recibe información
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
    def activar_busqueda(self):
        # Activa el modo de búsqueda publicando True en el topic
        msg = Bool()
        msg.data = True
        self.busqueda_pub.publish(msg)
        self.get_logger().info('Búsqueda activada')
        
    def desactivar_busqueda(self):
        # Desactiva el modo de búsqueda publicando False en el topic
        msg = Bool()
        msg.data = False
        self.busqueda_pub.publish(msg)
        self.get_logger().info('Búsqueda desactivada')
        
    def detener_robot(self):
        # Detiene completamente el robot enviando velocidades cero a todo
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
    
    def enviar_goal_nav2(self, x, y):
        # Envía un objetivo de navegación a Nav2
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Servidor Nav2 no disponible')
            return False
        
        # Cancelar navegación anterior si existe una en curso
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        
        # Construir mensaje de objetivo con coordenadas del mapa
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Goal enviado: ({x:.2f}, {y:.2f})')
        
        # Enviar objetivo de forma asíncrona y obtener handle para seguimiento
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=2.0)
        
        if send_goal_future.result() is not None:
            self.goal_handle = send_goal_future.result()
            if self.goal_handle.accepted:
                self.nav_result_future = self.goal_handle.get_result_async()
                return True
        
        return False
        
    def ejecutar_busqueda(self):
        # Función principal que coordina todo el proceso de búsqueda del tesoro
        self.activar_busqueda()
        time.sleep(2)  # Esperar a que se active el sistema de búsqueda
        
        self.tiempo_inicio = time.time()
        goal_enviado = False
        
        # Bucle principal de búsqueda
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            
            tiempo_transcurrido = time.time() - self.tiempo_inicio
            
            # Verificar si se excedió el tiempo límite
            if tiempo_transcurrido > self.tiempo_limite:
                self.get_logger().info('Tiempo agotado')
                break
            
            # Esperar hasta tener todos los datos necesarios
            if self.robot_x is None or self.distancia_x is None or self.distancia_y is None:
                continue
            
            # Verificar si el tesoro está lo suficientemente cerca
            if self.distancia_z is not None and self.distancia_z < self.distancia_objetivo:
                self.get_logger().info(f'Tesoro encontrado. Tiempo: {tiempo_transcurrido:.2f}s')
                self.encontrado = True
                break
            
            if not goal_enviado:
                # Calcular posición absoluta del tesoro: posición_robot + vector_distancia
                tesoro_x = self.robot_x + self.distancia_x
                tesoro_y = self.robot_y + self.distancia_y
                
                self.get_logger().info(f'Tesoro calculado en: ({tesoro_x:.2f}, {tesoro_y:.2f})')
                if self.enviar_goal_nav2(tesoro_x, tesoro_y):
                    goal_enviado = True
            else:
                # Gestión de reintento si Nav2 falla o aborta la navegación
                if self.nav_result_future is not None and self.nav_result_future.done():
                    result = self.nav_result_future.result()
                    if result and result.status in [5, 6]:  # Abortado o cancelado
                        self.get_logger().warn('Nav2 abortó, reintentando...')
                        
                        # Recalcular posición del tesoro con datos actualizados
                        tesoro_x = self.robot_x + self.distancia_x
                        tesoro_y = self.robot_y + self.distancia_y
                        self.enviar_goal_nav2(tesoro_x, tesoro_y)
            
            time.sleep(0.1)
        
        # Limpieza al finalizar la búsqueda
        self.detener_robot()
        self.desactivar_busqueda()

def main(args=None):
    rclpy.init(args=args)
    node = BusquedaTesoroNode()
    
    try:
        node.ejecutar_busqueda()
    except KeyboardInterrupt:
        pass
    finally:
        # Asegurar que el robot se detenga y la búsqueda se desactive al salir
        node.detener_robot()
        node.desactivar_busqueda()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
