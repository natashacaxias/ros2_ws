#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleController2(Node):
    """
    Controlador inspirado no sim_1.sce:
    - Objetivo: levar o ponto 'd' à frente da tartaruga até a origem (0,0).
    - Publica cmd_vel (linear.x, angular.z) com saturação em v_trans_max e v_rot_max.
    - Assina a pose para fechar a malha.
    """

    def __init__(self):
        super().__init__('turtle_controller2')

        # Parâmetros (altere se quiser reproduzir o sim_1.sce com outros valores)
        self.declare_parameter('d', 0.50)              # distância do ponto à frente (m)
        self.declare_parameter('v_trans_max', 1.0)     # m/s (sim_1.sce)
        self.declare_parameter('v_rot_max', math.pi/2) # rad/s (sim_1.sce)

        # Ganhos do controlador (proporcionais simples e estáveis p/turtlesim)
        self.declare_parameter('k_v', 0.8)   # ganho linear
        self.declare_parameter('k_wy', 2.0)  # ganho sobre e_y
        self.declare_parameter('k_wt', 0.5)  # ganho extra sobre theta (ajuda a orientar)

        self.d = float(self.get_parameter('d').value)
        self.v_trans_max = float(self.get_parameter('v_trans_max').value)
        self.v_rot_max = float(self.get_parameter('v_rot_max').value)
        self.k_v = float(self.get_parameter('k_v').value)
        self.k_wy = float(self.get_parameter('k_wy').value)
        self.k_wt = float(self.get_parameter('k_wt').value)

        # Publisher e Subscriber
        self.pub_cmd = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)

        # Timer de controle (20 Hz ~ 0.05 s)
        self.timer = self.create_timer(0.05, self.control_loop)

        # Estados locais
        self.pose = None
        self.goal = (0.0, 0.0)   # objetivo: origem
        self.arrived = False

        self.get_logger().info('turtle_controller2 iniciado (controle por pose).')

    def pose_cb(self, msg: Pose):
        self.pose = msg

    def saturate(self, val, lim):
        return max(-lim, min(lim, val))

    def control_loop(self):
        if self.pose is None:
            return

        x = float(self.pose.x)
        y = float(self.pose.y)
        theta = float(self.pose.theta)

        # Ponto à frente a distância d no referencial global
        x_front = x + self.d * math.cos(theta)
        y_front = y + self.d * math.sin(theta)

        # Erro até a origem (0,0), expresso no referencial do robô
        # e = R(-theta) * ([x_front; y_front] - [0;0])
        ex =  math.cos(theta) * x_front + math.sin(theta) * y_front
        ey = -math.sin(theta) * x_front + math.cos(theta) * y_front

        # Controle proporcional simples:
        # - v tenta reduzir ex (aproximar o ponto à frente da origem ao longo do eixo x do robô)
        # - w tenta reduzir ey (alinhar lateralmente) e também reduz orientacao absoluta
        v = - self.k_v  * ex
        w = - self.k_wy * ey - self.k_wt * theta

        # Saturações conforme sim_1.sce
        v = self.saturate(v, self.v_trans_max)
        w = self.saturate(w, self.v_rot_max)

        # Critério simples de "chegada"
        if math.hypot(x_front, y_front) < 0.05:
            self.arrived = True

        cmd = Twist()
        if self.arrived:
            # parou!
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Objetivo atingido: ponto à frente está na origem. Parando.')
        else:
            cmd.linear.x = v
            cmd.angular.z = w

        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

