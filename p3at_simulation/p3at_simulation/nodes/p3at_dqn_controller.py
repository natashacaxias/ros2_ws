import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import torch
import numpy as np
from p3at_simulation.dqn_agent import QNetwork
from ament_index_python.packages import get_package_share_directory
import os

class DQNController(Node):
    def __init__(self):
        super().__init__('dqn_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.loop)
        self.model = QNetwork(6, 6)  # ajustar dimensões

        # Caminho correto para o arquivo instalado
        share_dir = get_package_share_directory('p3at_simulation')
        model_path = os.path.join(share_dir, 'data', 'Q_model.pth')

        # Carrega o modelo
        self.model.load_state_dict(torch.load(model_path))
        self.model.eval()

        self.pose = np.zeros(3)
        self.goal = np.array([2.0, 2.0])

    def loop(self):
        erro = self.goal - self.pose[:2]
        dist = np.linalg.norm(erro)
        state = np.concatenate([self.pose, erro, [dist]])
        state_t = torch.FloatTensor(state).unsqueeze(0)
        q_vals = self.model(state_t)
        action_idx = q_vals.argmax().item()
        v = [0.0, 0.1, 0.2, 0.3][action_idx // 3]
        w = [-0.5, 0.0, 0.5][action_idx % 3]

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)

        dt = 0.1
        self.pose[0] += v * np.cos(self.pose[2]) * dt
        self.pose[1] += v * np.sin(self.pose[2]) * dt
        self.pose[2] += w * dt

def main(args=None):
    rclpy.init(args=args)
    node = DQNController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
