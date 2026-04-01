#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import random
import time
from tf_transformations import euler_from_quaternion

N_WAYPOINTS = 3
DESORIENTATION_TOLERANCE = 0.1
DIST_TOLERANCE = 0.3

class PID_TurtleController(Node):
    # Construtor
    def __init__(self):
        super().__init__("PID_turtle_controller")

        # Publisher — mesmo tópico do arquivo base
        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel_unstamped',
            10
        )

        # Subscriber de odometria (equivalente ao /turtle1/pose)
        self.subscription_ = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            10
        )

        # Pose atual (x, y, theta extraídos da odometria)
        self.x = None
        self.y = None
        self.theta = None

        # Timer para o loop de controle (mesmo intervalo do arquivo base: 0.5s → mantido em 0.1s para o PID)
        self.timer = self.create_timer(0.1, self.PID_control_loop)

        # Cria waypoints aleatórios
        self.createWayPoints()

        # Waypoint atual
        self.currentWp = 0

        # Integral dos erros
        self.I_theta = 0.0
        self.I_dist = 0.0

        # Derivada dos erros
        self.D_theta = 0.0
        self.D_dist = 0.0

        # Erro anterior
        self.past_dist_error = 0.0
        self.past_theta_error = 0.0

        # Tempo anterior
        self.last_time = time.monotonic()

        # Flag de conclusão
        self.isDone = False

    def createWayPoints(self):
        """Gera N_WAYPOINTS aleatórios (ajuste o range conforme o ambiente do seu robô)."""
        self.waypoints = [
            (
                random.uniform(-3.0, 3.0),
                random.uniform(-3.0, 3.0)
            ) for _ in range(N_WAYPOINTS)
        ]
        self.get_logger().info(f"{N_WAYPOINTS} pontos definidos:")
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f"  Ponto {i}: x={wp[0]:.2f}, y={wp[1]:.2f}")

    def pose_callback(self, msg: Odometry):
        """Extrai x, y e yaw da mensagem de odometria."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Converte quaternion → ângulo de Euler (yaw)
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.theta = yaw

    def PID_control_loop(self, Kp=1.0, Ki=0.1, Kd=0.1):
        """Loop de controle PID — equivalente direto ao do turtlesim."""

        # Aguarda a primeira leitura de odometria
        if self.x is None or self.isDone:
            return

        if self.currentWp < len(self.waypoints):
            x_goal, y_goal = self.waypoints[self.currentWp]
            dx = x_goal - self.x
            dy = y_goal - self.y
            error_dist = math.sqrt(dx**2 + dy**2)

            dt = time.monotonic() - self.last_time
            if dt <= 0:
                return

            # Derivada e integral da distância
            self.D_dist = (error_dist - self.past_dist_error) / dt
            self.I_dist += error_dist * dt

            # Waypoint atingido?
            if error_dist < DIST_TOLERANCE:
                self.get_logger().info(
                    f"Objetivo ({x_goal:.2f}, {y_goal:.2f}) alcançado!"
                )
                self.currentWp += 1
                return

            msg = Twist()
            theta_goal = math.atan2(dy, dx)
            theta_error = theta_goal - self.theta

            # Normaliza o erro angular para [-π, π]
            theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))

            # Derivada e integral do ângulo
            self.D_theta = (theta_error - self.past_theta_error) / dt
            self.I_theta += theta_error * dt

            # Prioriza rotação antes de avançar (igual ao turtlesim)
            if abs(theta_error) > DESORIENTATION_TOLERANCE:
                msg.angular.z = Kp * theta_error + Ki * self.I_theta + Kd * self.D_theta
                msg.linear.x = 0.0
            else:
                msg.angular.z = 0.0
                msg.linear.x = Kp * error_dist + Ki * self.I_dist + Kd * self.D_dist

            # Atualiza parâmetros anteriores
            self.past_dist_error = error_dist
            self.past_theta_error = theta_error
            self.last_time = time.monotonic()

            # Publica no tópico do robô real
            self.publisher_.publish(msg)

        else:
            self.isDone = True
            self.publisher_.publish(Twist())  # Para o robô
            self.get_logger().info("Todos os pontos foram alcançados!")


def main(args=None):
    rclpy.init(args=args)
    node = PID_TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()