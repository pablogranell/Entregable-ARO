import rclpy
from rclpy.node import Node
from minimal_interfaces.srv import ComandoNavegacion
import sys


class ClienteComandos(Node):
    def __init__(self):
        super().__init__('cliente_comandos')
        self.cliente = self.create_client(ComandoNavegacion, 'comando_navegacion')
        
        while not self.cliente.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servidor de comandos...')
        
        self.get_logger().info('Cliente de comandos conectado')

    def enviar_comando(self, comando):
        """Envía un comando al servidor y espera la respuesta"""
        request = ComandoNavegacion.Request()
        request.comando = comando
        
        self.get_logger().info(f'Enviando comando: {comando}')
        future = self.cliente.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Respuesta - Éxito: {response.exito}')
            self.get_logger().info(f'Mensaje: {response.mensaje}')
            return response
        else:
            self.get_logger().error('Error al llamar al servicio')
            return None


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("\nUso: ros2 run servicio_comandos cliente_comandos <comando>")
        print("Comandos disponibles:")
        print("  - Patrullar")
        print("  - GoToExit")
        print("\nEjemplo: ros2 run servicio_comandos cliente_comandos Patrullar\n")
        return
    
    comando = sys.argv[1]
    cliente = ClienteComandos()
    
    try:
        cliente.enviar_comando(comando)
    except KeyboardInterrupt:
        pass
    finally:
        cliente.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
