import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class QLearningController(Node):
    def __init__(self):
        super().__init__('qlearning_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        # === Q-learning params ===
        pkg_share = get_package_share_directory('p3at_simulation')
        q_path = os.path.join(pkg_share, 'data', 'Q_table.npy')
        self.Q_table = np.load(q_path, allow_pickle=True)
        self.vmax = 5
        self.rotmax = 2.5
        self.faixas_x = np.array([-1, -0.5, -0.25, -0.125, -0.0625,
                                  0, 0.0625, 0.125, 0.25, 0.5, 1])
        self.faixas_y = self.faixas_x.copy()
        self.v_valores = np.array([0, 0.05, 0.1, 0.15, 0.2,
                                   0.25, 0.3, 0.4, 0.5, 0.75, 1.0]) * self.vmax
        self.rot_valores = np.array([0, 0.05, 0.1, 0.2, 0.5, 1.0]) * self.rotmax
        self.num_acoes_v = len(self.v_valores)
        self.num_acoes_rot = len(self.rot_valores)

        # destino fixo ou sorteado
        self.destino = np.array([[2.0], [2.0]])
        self.pose = np.array([[0.0], [0.0], [0.0]])  # x,y,theta

    def control_loop(self):
        # calcula erro
        E = self.destino - self.pose[0:2, :]
        cs, ss = np.cos(self.pose[2, 0]), np.sin(self.pose[2, 0])
        Rot = np.array([[cs, ss], [-ss, cs]])
        Er = np.vstack((Rot @ E, self.pose[2, :]))

        dist_alvo = np.linalg.norm(Er[0:2, :])
        if dist_alvo < 0.1:
            self.get_logger().info("Alvo alcançado, parando!")
            msg = Twist()  # tudo zero
            self.publisher_.publish(msg)
            return

        erros_norm = Er[0:2, 0] / dist_alvo if dist_alvo > 1e-3 else np.array([0, 0])
        idx_x = np.sum(erros_norm[0] > self.faixas_x)
        idx_y = np.sum(erros_norm[1] > self.faixas_y)

        # ação da Q-table
        acao = np.argmax(self.Q_table[idx_x, idx_y, :])
        idx_v = acao // self.num_acoes_v
        idx_rot = acao % self.num_acoes_rot
        v = self.v_valores[idx_v] * erros_norm[0]
        rot = self.rot_valores[idx_rot] * erros_norm[1]

        # publica comando Twist
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(rot)
        self.publisher_.publish(msg)

        # atualiza pose simulada (só pra debug interno)
        dt = 0.1
        self.pose[0] += dt * v * np.cos(self.pose[2, 0])
        self.pose[1] += dt * v * np.sin(self.pose[2, 0])
        self.pose[2] = (self.pose[2] + dt * rot) % (2*np.pi)

        self.get_logger().info(f"x={self.pose[0,0]:.2f}, y={self.pose[1,0]:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = QLearningController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
