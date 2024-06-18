import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from pynput import keyboard
import sys
import os

class RobotControlNode(Node):

    def __init__(self):
        super().__init__('robot_control_node')

        # Variáveis globais para armazenar a velocidade
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.running = True
        self.key_pressed = None

        # Criar um publisher
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Criar serviços
        self.stop_srv = self.create_service(Empty, 'stop_robot', self.stop_robot_service)
        self.shutdown_srv = self.create_service(Empty, 'shutdown_robot', self.shutdown_robot_service)

        # Configurar o listener do teclado
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Configurar o timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Limpa o terminal
        self.clear_screen()

        # Imprime a mensagem inicial
        print("Aperte W para frente, S para trás, A para esquerda, D para direita, Q para sair.")

    def clear_screen(self):
        if os.name == 'nt':
            os.system('cls')
        else:
            os.system('clear')

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.key_pressed = 'w'
            elif key.char == 's':
                self.key_pressed = 's'
            elif key.char == 'a':
                self.key_pressed = 'a'
            elif key.char == 'd':
                self.key_pressed = 'd'
            elif key.char == 'q':
                self.stop_robot()
                self.shutdown_robot()
        except AttributeError:
            pass

    def on_release(self, key):
        if key == keyboard.Key.esc:
            self.running = False
        self.key_pressed = None

    def stop_robot(self):
        self.key_pressed = None
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.get_logger().info("Parada de emergência ativada!")

    def stop_robot_service(self, request, response):
        self.stop_robot()
        return response

    def shutdown_robot_service(self, request, response):
        self.shutdown_robot()
        return response

    def shutdown_robot(self):
        self.running = False
        self.get_logger().info("Desligando o robô...")
        rclpy.shutdown()

    def timer_callback(self):
        if not self.running:
            self.destroy_node()
            return

        # Atualiza as velocidades com base na tecla pressionada
        if self.key_pressed:
            if self.key_pressed == 'w':
                self.linear_speed = 2.0
            elif self.key_pressed == 's':
                self.linear_speed = -2.0
            elif self.key_pressed == 'a':
                self.angular_speed = 2.0
            elif self.key_pressed == 'd':
                self.angular_speed = -2.0
        else:
            # Se nenhum botão estiver pressionado, pare o robô
            self.linear_speed = 0.0
            self.angular_speed = 0.0

        # Imprime as velocidades atuais
        sys.stdout.write(f"\rVelocidade linear: {self.linear_speed}, Velocidade angular: {self.angular_speed}")
        sys.stdout.flush()

        # Publica a velocidade linear e angular
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.velocity_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()