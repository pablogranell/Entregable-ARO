#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import time
import math

class BusquedaTesoroNode(Node):
    def __init__(self):
        super().__init__('busqueda_tesoro_node')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.busqueda_pub = self.create_publisher(Bool, '/busquedaTesoro', 10)
        
        # Suscripción a la distancia del tesoro
        self.distancia_sub = self.create_subscription(
            Vector3,
            '/distanciaTesoro',
            self.distancia_callback,
            10
        )
        
        # Suscripción a la odometría para conocer la posición actual
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Cliente de acción para Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Variables básicas
        self.distancia_x = None
        self.distancia_y = None
        self.distancia_z = None
        self.robot_x = None
        self.robot_y = None
        
        # Control
        self.tiempo_inicio = None
        self.tiempo_limite = 90
        self.distancia_objetivo = 0.5
        self.encontrado = False
        
        # Nav2
        self.goal_handle = None
        self.nav_result_future = None
        
    def distancia_callback(self, msg):
        self.distancia_x = msg.x
        self.distancia_y = msg.y
        self.distancia_z = msg.z
    
    def odom_callback(self, msg):
        """Callback para obtener la posición actual del robot"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
    def activar_busqueda(self):
        msg = Bool()
        msg.data = True
        self.busqueda_pub.publish(msg)
        self.get_logger().info('Búsqueda activada')
        
    def desactivar_busqueda(self):
        msg = Bool()
        msg.data = False
        self.busqueda_pub.publish(msg)
        self.get_logger().info('Búsqueda desactivada')
        
    def detener_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        
    def mover_robot(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
    
    def enviar_goal_nav2(self, x, y):
        """Envía un goal a Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Servidor Nav2 no disponible')
            return False
        
        # Cancelar goal anterior si existe
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Goal enviado: ({x:.2f}, {y:.2f})')
        
        # Enviar y obtener handle
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=2.0)
        
        if send_goal_future.result() is not None:
            self.goal_handle = send_goal_future.result()
            if self.goal_handle.accepted:
                self.nav_result_future = self.goal_handle.get_result_async()
                return True
        
        return False
        
    def ejecutar_busqueda(self):
        self.activar_busqueda()
        time.sleep(2)
        
        self.tiempo_inicio = time.time()
        self.get_logger().info('=== INICIO DE BÚSQUEDA ===')
        
        goal_enviado = False
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            
            tiempo_transcurrido = time.time() - self.tiempo_inicio
            
            # Timeout
            if tiempo_transcurrido > self.tiempo_limite:
                self.get_logger().info('TIEMPO AGOTADO')
                break
            
            # Esperar datos
            if self.robot_x is None or self.distancia_x is None or self.distancia_y is None:
                continue
            
            # ¿Llegamos?
            if self.distancia_z is not None and self.distancia_z < self.distancia_objetivo:
                self.get_logger().info(f'¡TESORO ENCONTRADO! Tiempo: {tiempo_transcurrido:.2f}s')
                self.encontrado = True
                break
            
            # Enviar goal UNA sola vez
            if not goal_enviado:
                # El tesoro está en: posición_robot + vector_distancia
                tesoro_x = self.robot_x + self.distancia_x
                tesoro_y = self.robot_y + self.distancia_y
                
                self.get_logger().info(f'Tesoro calculado en: ({tesoro_x:.2f}, {tesoro_y:.2f})')
                if self.enviar_goal_nav2(tesoro_x, tesoro_y):
                    goal_enviado = True
            else:
                # Verificar si Nav2 abortó
                if self.nav_result_future is not None and self.nav_result_future.done():
                    result = self.nav_result_future.result()
                    if result and result.status in [5, 6]:  # ABORTED o CANCELED
                        self.get_logger().warn('Nav2 abortó, reintentando...')
                        
                        # Recalcular con posición actual
                        tesoro_x = self.robot_x + self.distancia_x
                        tesoro_y = self.robot_y + self.distancia_y
                        self.enviar_goal_nav2(tesoro_x, tesoro_y)
            
            time.sleep(0.1)
        
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
        node.detener_robot()
        node.desactivar_busqueda()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
