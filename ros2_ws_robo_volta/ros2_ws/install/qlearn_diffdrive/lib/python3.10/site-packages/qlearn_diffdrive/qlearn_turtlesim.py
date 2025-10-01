import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List
import random, time

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

class RobotQLearningV2ROS(Node):
    def __init__(self):
        super().__init__('qlearn_turtlesim_v2')

        # ======= Parâmetros FIÉIS ao V2 do professor =======  (:contentReference[oaicite:1]{index=1})
        self.d = 0.5
        self.vmax = 0.5
        self.rotmax = np.pi / 8
        self.h = 0.01                    # mantido por fidelidade (aqui usamos dt)
        self.caso = 2                    # ROS2 = caso 2 (atualiza pose e recalcula erro)
        self.alfa = 0.1
        self.gama = 0.9
        self.epsilon_inicial = 1.0
        self.epsilon_min = 0.05
        self.taxa_decaimento = 0.999
        self.num_episodios = 5000
        self.passos_por_episodio = 200
        self.dist_tolerancia = 0.2       # não usada para término; meta é o centro

        # faixas refinadas (lado /2 perto do centro)
        self.faixas_x = np.array([-1, -0.5, -0.25, -0.125, -0.0625, 0,
                                  0.0625, 0.125, 0.25, 0.5, 1], dtype=float)
        self.faixas_y = self.faixas_x.copy()
        self.centro_x = int(np.where(self.faixas_x == 0)[0][0] + 1)  # 1-based
        self.centro_y = int(np.where(self.faixas_y == 0)[0][0] + 1)

        # ações SÓ POSITIVAS, escaladas por vmax/rotmax (vetores do professor)
        self.v_valores = np.array([0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.75, 1.0]) * self.vmax
        self.rot_valores = np.array([0, 0.05, 0.1, 0.2, 0.5, 1.0]) * self.rotmax

        self.num_acoes_v = len(self.v_valores)
        self.num_acoes_rot = len(self.rot_valores)
        self.num_acoes = self.num_acoes_v * self.num_acoes_rot

        self.num_estados_x = len(self.faixas_x)
        self.num_estados_y = len(self.faixas_y)
        self.Q_table = np.zeros((self.num_estados_x, self.num_estados_y, self.num_acoes))
        self.epsilon = self.epsilon_inicial

        # ======= ROS2 I/O =======
        self.pub_cmd = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self._pose_cb, 10)
        self.cli_teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pose = None
        self.dt = 0.05

        self.destino = np.array([4.0, 3.0], dtype=float)  # mesmo alvo do V2
        self._started = False
        self.create_timer(0.1, self._maybe_start)

    # ---------- callbacks ----------
    def _pose_cb(self, msg: Pose):
        self.pose = msg

    def _maybe_start(self):
        if self.pose is None or self._started:
            return
        self._started = True
        self.get_logger().info('Treinando (V2, caso=2, fiel ao arquivo do professor).')
        historico = self.treinar()
        np.save('/home/robot/q_table_final.npy', self.Q_table)
        self.get_logger().info('Q-table salva em /home/robot/q_table_final.npy')
        self.plotar_progresso(historico)
        rclpy.shutdown()

    # ========== mesmas funções do V2 (nomes/fluxo) ==========
    def calc_estados_discretos(self, erro_bruto: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        dist = float(np.hypot(erro_bruto[0], erro_bruto[1]))
        if dist < 1e-2:
            nx, ny = 0.0, 0.0
        else:
            nx, ny = erro_bruto[0]/dist, erro_bruto[1]/dist
        ix = np.sum(nx > self.faixas_x) + 1
        iy = np.sum(ny > self.faixas_y) + 1
        ix = max(1, min(ix, self.num_estados_x))
        iy = max(1, min(iy, self.num_estados_y))
        return np.array([ix, iy], dtype=int), np.array([nx, ny], dtype=float), dist

    def escolher_acao(self, estado_discreto: np.ndarray) -> int:
        if random.random() < self.epsilon:
            return int(round(random.random() * (self.num_acoes - 1)) + 1)  # 1-based
        ix, iy = estado_discreto[0], estado_discreto[1]
        return int(np.argmax(self.Q_table[ix-1, iy-1, :]) + 1)              # 1-based

    def mapear_acao_para_velocidades(self, acao: int, erros_norm: np.ndarray) -> Tuple[float, float]:
        idx_v = (acao - 1) // self.num_acoes_rot
        idx_r = (acao - 1) %  self.num_acoes_rot
        v = float(self.v_valores[idx_v] * erros_norm[0])   # sinal vem do erro normalizado
        w = float(self.rot_valores[idx_r] * erros_norm[1])
        return v, w

    # --- funções do V2 adaptadas ao ROS (CASO 2) ---
    def calcular_erro_inicial(self, destino: np.ndarray, pose_vec: np.ndarray) -> np.ndarray:
        # E = destino - pose (no fixo)
        E = destino[:2] - pose_vec[:2]
        cs, ss = np.cos(pose_vec[2]), np.sin(pose_vec[2])
        Rot = np.array([[cs, ss], [-ss, cs]])  # rotação do fixo p/ robô
        Er_xy = Rot @ E - np.array([self.d, 0.0])
        return np.array([Er_xy[0], Er_xy[1], pose_vec[2]], dtype=float)

    def simular_acao_caso2(self, v: float, w: float) -> np.ndarray:
        # publica Twist e espera dt; depois calcula Er com a pose real
        cmd = Twist(); cmd.linear.x = v; cmd.angular.z = w
        self.pub_cmd.publish(cmd)
        t0 = time.time()
        while time.time() - t0 < self.dt:
            rclpy.spin_once(self, timeout_sec=0.0)
        pose_vec = np.array([self.pose.x, self.pose.y, self.pose.theta], dtype=float)
        return self.calcular_erro_inicial(self.destino, pose_vec)

    def calcular_recompensa(self, estado_discreto: np.ndarray, estado_prox: np.ndarray) -> float:
        # base -1; +100 se chegou ao centro; -5 se afastou (soma índices maior)
        if (estado_discreto[0] == self.centro_x and estado_discreto[1] == self.centro_y):
            return 100.0
        if np.sum(estado_prox) > np.sum(estado_discreto):
            return -5.0
        return -1.0

    def atualizar_tabela_q(self, estado: np.ndarray, acao: int, R: float, estado2: np.ndarray):
        ix, iy = estado[0]-1, estado[1]-1
        ix2, iy2 = estado2[0]-1, estado2[1]-1
        aidx = acao - 1
        Qsa = self.Q_table[ix, iy, aidx]
        Qmax2 = np.max(self.Q_table[ix2, iy2, :])
        self.Q_table[ix, iy, aidx] = Qsa + self.alfa * (R + self.gama * Qmax2 - Qsa)

    # ========== laço de treino (mesmo fluxo do V2) ==========
    def treinar(self) -> List[Tuple[int, int]]:
        historico: List[Tuple[int, int]] = []

        for ep in range(1, self.num_episodios + 1):
            # reset pose perto do centro
            if self.cli_teleport.wait_for_service(timeout_sec=1.0):
                req = TeleportAbsolute.Request(); req.x, req.y, req.theta = 5.5, 5.5, 0.0
                self.cli_teleport.call_async(req); rclpy.spin_once(self, timeout_sec=0.05)

            pose_vec = np.array([self.pose.x, self.pose.y, self.pose.theta], dtype=float)
            Er = self.calcular_erro_inicial(self.destino, pose_vec)
            estado, en, _ = self.calc_estados_discretos(Er)

            for st in range(1, self.passos_por_episodio + 1):
                acao = self.escolher_acao(estado)
                v, w = self.mapear_acao_para_velocidades(acao, en)
                Er_novo = self.simular_acao_caso2(v, w)
                estado2, en, _ = self.calc_estados_discretos(Er_novo)
                R = self.calcular_recompensa(estado, estado2)
                self.atualizar_tabela_q(estado, acao, R, estado2)
                estado = estado2

                # término quando chega ao centro (meta do V2) ou bate limite
                chegou = (estado[0] == self.centro_x and estado[1] == self.centro_y)
                if chegou or st == self.passos_por_episodio:
                    tag = 'CHEGOU' if chegou else 'LIMITE'
                    self.get_logger().info(f'ep {ep:4d}  steps {st:3d}  eps {self.epsilon:.3f}  {tag}')
                    break

            historico.append((ep, st))
            self.epsilon = max(self.epsilon_min, self.epsilon * self.taxa_decaimento)

        return historico

    # ====== igual ao V2: gráficos de progresso e Q-max ======
    def plotar_progresso(self, historico: List[Tuple[int, int]]):
        episodios = [h[0] for h in historico]
        passos = [h[1] for h in historico]

        plt.figure(figsize=(15, 5))
        plt.subplot(1, 3, 1)
        plt.plot(episodios, passos, alpha=0.6)
        plt.xlabel('Episódio'); plt.ylabel('Passos'); plt.title('Progresso'); plt.grid(True)

        window = 100
        if len(passos) >= window:
            mm = np.convolve(passos, np.ones(window)/window, mode='valid')
            plt.subplot(1, 3, 2)
            plt.plot(range(window-1, len(passos)), mm)
            plt.xlabel('Episódio'); plt.ylabel('Média (100)'); plt.title('Tendência'); plt.grid(True)

        plt.subplot(1, 3, 3)
        Q_max = np.max(self.Q_table, axis=2)
        im = plt.imshow(Q_max, cmap='viridis', origin='lower')
        plt.colorbar(im); plt.xlabel('Estado Y'); plt.ylabel('Estado X'); plt.title('Q_max por Estado')
        plt.plot(self.centro_y-1, self.centro_x-1, 'r*', markersize=15, label='Objetivo'); plt.legend()
        plt.tight_layout(); plt.show()

def main():
    rclpy.init()
    node = RobotQLearningV2ROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

