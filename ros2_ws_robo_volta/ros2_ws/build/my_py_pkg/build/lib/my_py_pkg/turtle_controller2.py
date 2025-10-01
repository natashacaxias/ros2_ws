#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleController2(Node):
    """
    Controlador inspirado no sim_1.sce:
    - Objetivo: levar o ponto 'd' à frente da tartaruga até (0,0).
    - Saturações iguais às do script: v_trans_max=1.0, v_rot_max=pi/2.
    """

    def __init__(self):
        super().__init__('turtle_controller2')

        # === MESMOS VALORES DO sim_1.sce ===
        self.d = 0.50
        self.v_trans_max = 1.0            # m/s
        self.v_rot_max = math.pi / 2.0    # rad/s

        # Ganhos simples e estáveis p/ reproduzir a regulação do ponto à frente
        # (Ajuste fino opcional: k_v ↑ acelera aproximação, k_wy/k_wt ↑ acelera alinhamento)
        self.k_v  = 0.8
        self.k_wy = 2.0
        self.k_wt = 0.5

        self.pub_cmd = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)

        # Timer de controle ~50 Hz (passo de integração discreta)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.pose = None
        self.arrived = False
        self.get_logger().info('turtle_controller2 com d=0.50, v_max=1.0 m/s, w_max=pi/2 rad/s')

    def pose_cb(self, msg: Pose):
        self.pose = msg

    @staticmethod
    def sat(val, lim):
        return max(-lim, min(lim, val))

    def control_loop(self):
        if self.pose is None:
            return

        x = float(self.pose.x)
        y = float(self.pose.y)
        th = float(self.pose.theta)

        # Ponto à frente (mesma definição do .sce): (x + d cos th, y + d sin th)
        xf = x + self.d * math.cos(th)
        yf = y + self.d * math.sin(th)

        # Erro do ponto à frente até a origem expresso no frame do robô
        ex =  math.cos(th)*xf + math.sin(th)*yf     # eixo x do robô
        ey = -math.sin(th)*xf + math.cos(th)*yf     # eixo y do robô

        # Controle proporcional (analogia ao regulador do .sce)
        v = - self.k_v  * ex
        w = - self.k_wy * ey - self.k_wt * th

        # Saturações iguais às do sim_1.sce
        v = self.sat(v, self.v_trans_max)
        w = self.sat(w, self.v_rot_max)

        # Critério de chegada do ponto à frente à origem
        if math.hypot(xf, yf) < 0.05:
            self.arrived = True

        cmd = Twist()
        if self.arrived:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
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

