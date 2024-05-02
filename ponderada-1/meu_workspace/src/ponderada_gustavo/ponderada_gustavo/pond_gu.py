import rclpy
import asyncio
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, SetPen

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller') 
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  
        self.timer_period = 1
        self.side_count = 0 
        self.max_sides = 4  #  para 4 para desenhar um triângulo
        self.circle_drawn = False  # Flag para indicar se o círculo já foi desenhado

        # Clientes e serviços
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

    def straight(self):
        msg = Twist()
        msg.linear.x = 1.75  # Movimento linear para frente
        self.publisher.publish(msg)

    def curve(self):
        msg = Twist()
        msg.angular.z = 1.047  # Rotação de 60 graus
        self.publisher.publish(msg)

    def move_circular(self):
        msg = Twist()
        msg.linear.x = 1.0  # Velocidade linear constante
        msg.angular.z = 1.0  # Velocidade angular constante para girar em torno do próprio eixo
        self.publisher.publish(msg)

    async def set_pen_color(self, r, g, b):
        node = rclpy.create_node('set_pen_color')
        set_pen_client = node.create_client(SetPen, '/turtle1/set_pen')
        while not set_pen_client.wait_for_service(timeout_sec=1.0):
            print('Service not available, waiting again...')
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = 5  # Mantém a largura da linha inalterada
        request.off = 0    # Mantém a caneta ligada
        future = set_pen_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

    async def timer_callback(self):
        await self.set_pen_color(255, 165, 0)
        if self.side_count < self.max_sides * 2:  # Desenha dois triângulos
            if self.side_count % 3 == 0:
                self.straight()  # Movimento linear em lados ímpares
            else:
                self.curve()  # Curva em lados pares
            self.side_count += 1
        else:
            if not self.circle_drawn:
                # Desenho dos triângulos concluído, agora desenhamos um círculo dentro deles
                time.sleep(1)  # Espere um pouco para garantir que a tartaruga termine o movimento linear
                # Definimos a velocidade angular para fazer a tartaruga girar em torno de seu próprio eixo
                angular_velocity = 1.0
                # Definimos o passo de tempo para controlar a precisão do círculo
                dt = 0.1
                # Movemos a tartaruga em um círculo
                for _ in range(int(2 * math.pi / (angular_velocity * dt))):
                    self.move_circular()
                    time.sleep(dt)
                self.circle_drawn = True
                self.timer.cancel()  # Cancela o temporizador

    def spawn_circle(self, x, y, radius):
        node = rclpy.create_node('spawn_circle')
        client = node.create_client(Spawn, '/spawn')

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = 'circle_turtle'

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            print('Círculo criado')
        else:
            print('Falha ao criar círculo')

def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    tc.timer = tc.create_timer(tc.timer_period, tc.timer_callback)
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
