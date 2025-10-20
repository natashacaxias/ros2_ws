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

class P3ATDQNPolicy(Node):
    def __init__(self):
        super().__init__('p3at_dqn_policy')

        # Publisher e subscriber ROS
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_received = False

        # Timer de controle
        self.timer = self.create_timer(0.1, self.control_loop)

        # Diretório do pacote e modelos
        pkg_share = get_package_share_directory('p3at_qlearning_simulation_v2')
        model_path = os.path.join(pkg_share, 'dqn_politica.pt')
        meta_path = os.path.join(pkg_share, 'dqn_politica_meta.npz')

        if not os.path.exists(model_path):
            self.get_logger().error(f"❌ Modelo não encontrado: {model_path}")
            raise FileNotFoundError(model_path)

        self.get_logger().info(f"✅ Carregando modelo: {model_path}")
        self.get_logger().info(f"📘 Carregando metadados: {meta_path}")

        # Carrega metadados
        meta = np.load(meta_path, allow_pickle=True)
        try:
            parametros = json.loads(meta['parametros_json'].item())
        except Exception:
            parametros = json.loads(meta['parametros_json'])

        # Extrai parâmetros
        self.v_valores = np.array(parametros['v_valores'])
        self.rot_valores = np.array(parametros['rot_valores'])
        self.state_dim = int(parametros['state_dim'])
        self.action_dim = int(parametros['action_dim'])
        self.d = float(parametros['d'])

        # Inicializa modelo DQN
        self.model = DQN(self.state_dim, self.action_dim).to(DEVICE)
        self.model.load_state_dict(torch.load(model_path, map_location=DEVICE))
        self.model.eval()

        self.goal = np.array([4.0, 3.0])
        self.current_pose = np.zeros(3)
        self.goal_reached = False

        self.get_logger().info("🤖 Nó P3AT-DQN iniciado com sucesso!")
        self.get_logger().info(f"🎯 Alvo: {self.goal}")
        self.get_logger().info(f"⚙️  Dimensões: state={self.state_dim}, action={self.action_dim}")
        self.get_logger().info(f" ^z^y  ^o  Configuracoes: vel. trans.={self.v_valores}, vel. rot.={self.rot_valores}")

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose = np.array([pos.x, pos.y, theta])
        self.odom_received = True

    def calcular_erro_inicial(self, destino: np.ndarray, pose: np.ndarray) -> np.ndarray:
        E = destino[:2] - pose[:2]
        cs = np.cos(pose[2])
        ss = np.sin(pose[2])
        Rot = np.array([[cs, ss], [-ss, cs]])
        Er_xy = Rot @ E - np.array([self.d, 0])
        Er = np.array([Er_xy[0], Er_xy[1], pose[2]])
        return Er

    def calc_estados_continuos(self, erro_bruto: np.ndarray) -> Tuple[np.ndarray, float]:
        dist = np.sqrt(erro_bruto[0]**2 + erro_bruto[1]**2)
        if dist < 0.01:
            erros_norm = np.zeros(2)
        else:
            erros_norm = np.array([erro_bruto[0]/dist, erro_bruto[1]/dist])
        state = np.array([erros_norm[0], erros_norm[1], dist], dtype=np.float32)
        return state, dist

    def map_action(self, action_idx: int, state: np.ndarray) -> Tuple[float, float]:
        idx_v = action_idx // len(self.rot_valores)
        idx_rot = action_idx % len(self.rot_valores)
        v = float(self.v_valores[idx_v] * state[0])
        rot = float(self.rot_valores[idx_rot] * state[1])
        return v, rot

    def control_loop(self):
        if not self.odom_received:
            self.get_logger().warn("⏳ Aguardando odometria...")
            return

        Er = self.calcular_erro_inicial(self.goal, self.current_pose)
        state, dist = self.calc_estados_continuos(Er)
        state_t = torch.FloatTensor(state).unsqueeze(0).to(DEVICE)

        with torch.no_grad():
            qvals = self.model(state_t)
            action_idx = int(qvals.argmax(dim=1).item())

        v, rot = self.map_action(action_idx, state)

        # Publica no tópico /cmd_vel
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = rot
        self.cmd_pub.publish(msg)

        # Logs detalhados
        self.get_logger().info(
            f"🚗 Ação={action_idx} | v={v:.3f}, ω={rot:.3f} | Distância={dist:.2f} | "
            f"Pos=({self.current_pose[0]:.2f},{self.current_pose[1]:.2f}) | Alvo=({self.goal[0]},{self.goal[1]})"
        )

        if dist < 0.2:
            self.get_logger().info("🎯 Alvo alcançado! Parando robô.")
            self.parar_robo()
            self.goal_reached = True

    def parar_robo(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = P3ATDQNPolicy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Encerrado pelo usuário.")
        node.parar_robo()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
