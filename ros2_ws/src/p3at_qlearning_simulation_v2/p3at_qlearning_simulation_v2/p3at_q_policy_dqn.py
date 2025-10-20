#!/usr/bin/env python3
import numpy as np
np.float = float

import torch
import torch.nn as nn
import os
import json
from typing import Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
import tf_transformations

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# === REDE DQN ===
class DQN(nn.Module):
    def __init__(self, input_dim: int, output_dim: int, hidden: int = 256):
        super(DQN, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, output_dim)
        )

    def forward(self, x):
        return self.net(x)

# === CONTROLADOR MULTI-ROBÔ ===
class DualP3ATDQNPolicy(Node):
    def __init__(self):
        super().__init__('dual_p3at_dqn_policy')

        # ==============================
        # 1. CONFIGURAÇÕES DE MODELO
        # ==============================
        pkg_share = get_package_share_directory('p3at_qlearning_simulation_v2')
        model_path = os.path.join(pkg_share, 'dqn_politica.pt')
        meta_path = os.path.join(pkg_share, 'dqn_politica_meta.npz')

        if not os.path.exists(model_path):
            self.get_logger().error(f"❌ Modelo não encontrado: {model_path}")
            raise FileNotFoundError(model_path)

        meta = np.load(meta_path, allow_pickle=True)
        try:
            parametros = json.loads(meta['parametros_json'].item())
        except Exception:
            parametros = json.loads(meta['parametros_json'])

        self.v_valores = np.array(parametros['v_valores'])
        self.rot_valores = np.array(parametros['rot_valores'])
        self.state_dim = int(parametros['state_dim'])
        self.action_dim = int(parametros['action_dim'])
        self.d = float(parametros['d'])

        self.model = DQN(self.state_dim, self.action_dim).to(DEVICE)
        self.model.load_state_dict(torch.load(model_path, map_location=DEVICE))
        self.model.eval()

        # ==============================
        # 2. CONFIGURAÇÕES DE ROBÔS
        # ==============================
        self.goal1 = np.array([4.0, 3.0])
        self.goal2 = np.array([-4.0, -3.0])
        self.pose1 = np.zeros(3)
        self.pose2 = np.zeros(3)
        self.odom1_received = False
        self.odom2_received = False

        # ==============================
        # 3. TÓPICOS ROS
        # ==============================
        self.cmd_pub1 = self.create_publisher(Twist, '/p3at1/cmd_vel', 10)
        self.cmd_pub2 = self.create_publisher(Twist, '/p3at2/cmd_vel', 10)
        self.create_subscription(Odometry, '/p3at1/odom', self.odom1_cb, 10)
        self.create_subscription(Odometry, '/p3at2/odom', self.odom2_cb, 10)

        # Loop principal de controle (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("🤖 Controlador DQN para dois robôs iniciado!")
        self.get_logger().info(f"🎯 Robô 1 → alvo {self.goal1}")
        self.get_logger().info(f"🎯 Robô 2 → alvo {self.goal2}")

    # === CALLBACKS DE ODOM ===
    def odom1_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose1 = np.array([pos.x, pos.y, theta])
        self.odom1_received = True

    def odom2_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose2 = np.array([pos.x, pos.y, theta])
        self.odom2_received = True

    # === FUNÇÕES DE CÁLCULO ===
    def calcular_erro(self, destino, pose):
        E = destino[:2] - pose[:2]
        cs, ss = np.cos(pose[2]), np.sin(pose[2])
        Rot = np.array([[cs, ss], [-ss, cs]])
        Er_xy = Rot @ E - np.array([self.d, 0])
        return np.array([Er_xy[0], Er_xy[1], pose[2]])

    def calc_estados_continuos(self, Er):
        dist = np.hypot(Er[0], Er[1])
        erros_norm = np.zeros(2) if dist < 0.01 else Er[:2] / dist
        state = np.array([erros_norm[0], erros_norm[1], dist], dtype=np.float32)
        return state, dist

    def map_action(self, a_idx, state):
        idx_v = a_idx // len(self.rot_valores)
        idx_rot = a_idx % len(self.rot_valores)
        v = float(self.v_valores[idx_v] * state[0])
        rot = float(self.rot_valores[idx_rot] * state[1])
        return v, rot

    # === LOOP DE CONTROLE ===
    def control_loop(self):
        if not (self.odom1_received and self.odom2_received):
            self.get_logger().warn("⏳ Aguardando odometria de ambos os robôs...")
            return

        for i, (pose, goal, pub) in enumerate([
            (self.pose1, self.goal1, self.cmd_pub1),
            (self.pose2, self.goal2, self.cmd_pub2)
        ], start=1):
            Er = self.calcular_erro(goal, pose)
            state, dist = self.calc_estados_continuos(Er)
            state_t = torch.FloatTensor(state).unsqueeze(0).to(DEVICE)

            with torch.no_grad():
                qvals = self.model(state_t)
                a_idx = int(qvals.argmax(dim=1).item())

            v, rot = self.map_action(a_idx, state)
            msg = Twist()
            msg.linear.x, msg.angular.z = v, rot
            pub.publish(msg)

            self.get_logger().info(
                f"[P3AT{i}] Ação={a_idx} | v={v:.3f}, ω={rot:.3f} | Dist={dist:.2f} "
                f"| Pose=({pose[0]:.2f},{pose[1]:.2f}) → Goal=({goal[0]:.1f},{goal[1]:.1f})"
            )

            if dist < 0.25:
                self.get_logger().info(f"🎯 P3AT{i} chegou ao destino!")
                self.parar(pub)

    def parar(self, pub):
        msg = Twist()
        pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DualP3ATDQNPolicy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Encerrado pelo usuário.")
        node.parar(node.cmd_pub1)
        node.parar(node.cmd_pub2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
