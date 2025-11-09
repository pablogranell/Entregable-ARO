#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
import time
import math

class BusquedaTesoroNode(Node):
    def __init__(self):
        super().__init__('busqueda_tesoro_node')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.busqueda_pub = self.create_publisher(Bool, '/busquedaTesoro', 10)
        self.distancia_sub = self.create_subscription(
            Vector3,
            '/distanciaTesoro',
            self.distancia_callback,
            10
        )
        
        self.distancia_x = None
        self.distancia_y = None
        self.distancia_z = None
        self.distancia_anterior = None
        self.mejorando = False
        self.contador_estable = 0
        self.tiempo_inicio = None
        self.tiempo_limite = 90
        self.distancia_objetivo = 0.5
        self.encontrado = False
        
    def distancia_callback(self, msg):
        self.distancia_x = msg.x
        self.distancia_y = msg.y
        self.distancia_z = msg.z
        
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
        
    def ejecutar_busqueda(self):
        self.activar_busqueda()
        time.sleep(2)
        
        self.tiempo_inicio = time.time()
        self.get_logger().info('Iniciando búsqueda del tesoro...')
        
        ultimo_log = 0.0
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            
            tiempo_transcurrido = time.time() - self.tiempo_inicio
            tiempo_restante = self.tiempo_limite - tiempo_transcurrido
            
            if tiempo_restante <= 0:
                self.get_logger().info('TIEMPO AGOTADO - Búsqueda fallida')
                self.detener_robot()
                self.desactivar_busqueda()
                break
            
            if tiempo_transcurrido - ultimo_log >= 1.0:
                if self.distancia_z is not None:
                    self.get_logger().info(f'Tiempo: {tiempo_restante:.1f}s | Distancia: {self.distancia_z:.2f}m')
                else:
                    self.get_logger().info(f'Tiempo: {tiempo_restante:.1f}s | Sin datos')
                ultimo_log = tiempo_transcurrido
            
            if self.distancia_z is not None:
                if self.distancia_z < self.distancia_objetivo:
                    self.get_logger().info(f'TESORO ENCONTRADO en {tiempo_transcurrido:.2f} segundos')
                    self.detener_robot()
                    self.desactivar_busqueda()
                    self.encontrado = True
                    break
                
                # Estrategia simple: detectar si nos acercamos o alejamos
                if self.distancia_anterior is not None:
                    diferencia = self.distancia_z - self.distancia_anterior
                    
                    if diferencia < -0.01:  # Nos acercamos (umbral más sensible)
                        self.mejorando = True
                        self.contador_estable = 0
                        # Avanzar más rápido cuando nos acercamos
                        velocidad_linear = 0.22
                        velocidad_angular = 0.0
                    elif abs(diferencia) < 0.01:  # Distancia estable (umbral más sensible)
                        self.contador_estable += 1
                        if self.contador_estable > 5:
                            # Llevamos tiempo sin mejorar, girar MIENTRAS avanzamos para explorar
                            velocidad_linear = 0.12
                            velocidad_angular = 0.3  # Giro suave mientras avanza
                        else:
                            # Seguir avanzando
                            velocidad_linear = 0.18
                            velocidad_angular = 0.0
                    else:  # Nos alejamos
                        # Girar más fuerte para cambiar dirección, pero seguir avanzando un poco
                        velocidad_linear = 0.08
                        velocidad_angular = 0.6
                        self.mejorando = False
                        self.contador_estable = 0
                else:
                    # Primera iteración, empezar avanzando despacio mientras gira
                    velocidad_linear = 0.1
                    velocidad_angular = 0.3
                
                self.mover_robot(velocidad_linear, velocidad_angular)
                self.distancia_anterior = self.distancia_z
            else:
                self.mover_robot(0.0, 0.3)
            
            time.sleep(0.1)

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
