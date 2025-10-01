import rclpy
from rclpy.node import Node
import numpy as np, time
from typing import Tuple
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class ExecutePolicyV2(Node):
    def __init__(self):
        super().__init__('qlearn_exec_v2')

        # mesmos parâmetros do V2 (:contentReference[oaicite:2]{index=2})
        self.d = 0.5
        self.vmax = 0.5
        self.rotmax = np.pi/8
        self.faixas_x = np.array([-1,-0.5,-0.25,-0.125,-0.0625,0,0.0625,0.125,0.25,0.5,1], dtype=float)
        self.faixas_y = self.faixas_x.copy()
        self.centro_x = int(np.where(self.faixas_x == 0)[0][0] + 1)
        self.centro_y = int(np.where(self.faixas_y == 0)[0][0] + 1)
        self.v_valores = np.array([0,0.05,0.1,0.15,0.2,0.25,0.3,0.4,0.5,0.75,1.0]) * self.vmax
        self.rot_valores = np.array([0,0.05,0.1,0.2,0.5,1.0]) * self.rotmax
        self.num_acoes_rot = len(self.rot_valores)

        self.destino = np.array([4.0,3.0], dtype=float)
        self.dt = 0.05

        self.Q_table = np.load('/home/robot/q_table_final.npy')
        self.get_logger().info(f'Q-table carregada: shape={self.Q_table.shape}')

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self._pose_cb, 10)
        self.pose = None
        self.steps = 0
        self.running = True

        self.create_timer(self.dt, self._step)

    def _pose_cb(self, msg: Pose):
        self.pose = msg

    def _calc_estado(self, Er: np.ndarray):
        dist = float(np.hypot(Er[0], Er[1]))
        if dist < 1e-2: nx, ny = 0.0, 0.0
        else: nx, ny = Er[0]/dist, Er[1]/dist
        ix = np.sum(nx > self.faixas_x) + 1
        iy = np.sum(ny > self.faixas_y) + 1
        ix = max(1, min(ix, len(self.faixas_x)))
        iy = max(1, min(iy, len(self.faixas_y)))
        return np.array([ix, iy], dtype=int), np.array([nx, ny], dtype=float)

    def _step(self):
        if not self.running or self.pose is None: return
        pose_vec = np.array([self.pose.x, self.pose.y, self.pose.theta], dtype=float)

        # Er no frame do robô (igual ao V2)
        E = self.destino[:2] - pose_vec[:2]
        cs, ss = np.cos(pose_vec[2]), np.sin(pose_vec[2])
        Rot = np.array([[cs, ss], [-ss, cs]])
        Er_xy = Rot @ E - np.array([self.d, 0.0])
        Er = np.array([Er_xy[0], Er_xy[1], pose_vec[2]], dtype=float)

        estado, en = self._calc_estado(Er)

        # chegou ao CENTRO?
        if estado[0] == self.centro_x and estado[1] == self.centro_y:
            self.get_logger().info(f'Alvo (centro) atingido em {self.steps} passos.')
            self.pub.publish(Twist())
            self.running = False
            return

        # melhor ação (epsilon=0)
        ix, iy = estado[0]-1, estado[1]-1
        aidx = int(np.argmax(self.Q_table[ix, iy, :]))
        iv = aidx // self.num_acoes_rot
        ir = aidx %  self.num_acoes_rot
        v = float(self.v_valores[iv] * en[0])
        w = float(self.rot_valores[ir] * en[1])

        cmd = Twist(); cmd.linear.x = v; cmd.angular.z = w
        self.pub.publish(cmd)

        self.steps += 1
        if self.steps % 20 == 0:
            self.get_logger().info(f'steps={self.steps}  v={v:.3f}  w={w:.3f}')

        time.sleep(self.dt)

def main():
    rclpy.init()
    node = ExecutePolicyV2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()

