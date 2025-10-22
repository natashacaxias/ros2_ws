import numpy as np

# Corrigir problemas com Float do numpy antigo
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry  # Para /odom
import tf_transformations
import math
import os


class P3ATQPolicy(Node):
    def __init__(self):
        super().__init__('p3at_q_policy')
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Inicializa o timer (tempo de varredura ou tempo do laço de controle)
        timer_period = 0.10  # 10 HZ
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        #inicializa o flag de estados
        self.odom_received = False
                
        self.odom_sub_ = self.create_subscription(
        	Odometry,
        	'/odom',
        	self.odom_callback,
        	10
        )

        # Parâmetros do robô (DEVEM ser iguais aos usados no treinamento!)
        self.d = 0.5  # do arquivo Scilab
        self.vmax = 2.0  # do arquivo Scilab
        self.rotmax = 1.0  # do arquivo Scilab
        self.dist_tolerancia = 0.2  # do arquivo Scilab
        
        # Discretização (DEVE ser igual à do treinamento!)
        self.faixas_x = np.array([-1.0, -0.5, -0.25, -0.125, -0.0625, 0.0, 
                                   0.0625, 0.125, 0.25, 0.5, 1.0])
        self.faixas_y = np.array([-1.0, -0.5, -0.25, -0.125, -0.0625, 0.0, 
                                   0.0625, 0.125, 0.25, 0.5, 1.0])
        
        # Ações (DEVEM ser iguais às do treinamento!)
        # 11 velocidades x 6 rotações = 66 ações
        self.v_valores = np.array([-1.0, -0.75, -0.5, -0.25, 0, 0.25, 
                                   0.5, 0.75, 1.0, 1.25, 1.5]) * self.vmax
        self.rot_valores = np.array([-1.0, -0.5, 0, 0.5, 1.0, 1.5]) * self.rotmax
        
        # Dimensões da tabela Q
        self.num_estados_x = len(self.faixas_x)
        self.num_estados_y = len(self.faixas_y)
        
        self.num_acoes_v = len(self.v_valores)
        self.num_acoes_rot = len(self.rot_valores)
        self.num_acoes_total = self.num_acoes_v * self.num_acoes_rot
        
        # Carrega Q-table treinada
        self.declare_parameter('q_table_file', 'politica.npz')
        q_table_file = self.get_parameter('q_table_file').get_parameter_value().string_value
        
        try:
            # 1. Obter o diretório de instalação do pacote
            #pkg_share_dir = get_package_share_directory('p3at_qlearning_simulation_v2')
            # 2. Construir o caminho completo (assumindo que você o instalou na pasta 'data')
            #q_table_path = os.path.join(pkg_share_dir, 'data', q_table_file) 
        
            #self.Q_table = np.load(q_table_path) # Carrega o arquivo usando o caminho completo
            #self.get_logger().info(f"✅ Q-table carregada de: {q_table_path}")
            #self.get_logger().info(f"📊 Shape: {self.Q_table.shape}")
            #self.get_logger().info(f"📈 Valores não-zero: {np.count_nonzero(self.Q_table)}")
            #self.get_logger().info(f"📈 Min: {self.Q_table.min():.3f}, Max: {self.Q_table.max():.3f}")

            # 1. Obter o diretório de instalação do pacote
            pkg_share_dir = get_package_share_directory('p3at_qlearning_simulation_v2')
            # 2. Construir o caminho completo (assumindo que você o instalou na pasta 'data')
            q_table_path = os.path.join(pkg_share_dir, q_table_file)

            # Carregar o arquivo .npz (pode conter múltiplos arrays)
            npz_data = np.load(q_table_path)

            # Verificar quais chaves existem
            self.get_logger().info(f"🔑 Chaves encontradas na Q-table: {list(npz_data.keys())}")

            # Selecionar o primeiro array (ou troque o nome da chave, se souber)
            key_name = list(npz_data.keys())[0]
            self.Q_table = npz_data[key_name]

            # Logs informativos
            self.get_logger().info(f"✅ Q-table carregada de: {q_table_path}")
            self.get_logger().info(f"📊 Shape: {self.Q_table.shape}")
            self.get_logger().info(f"📈 Valores não-zero: {np.count_nonzero(self.Q_table)}")
            self.get_logger().info(f"📈 Min: {self.Q_table.min():.3f}, Max: {self.Q_table.max():.3f}")

        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar Q-table: {e}")
            raise

        # Objetivo
        self.declare_parameter('goal_x', 4.0)
        self.declare_parameter('goal_y', 8.0)
        goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.XF_M = np.array([goal_x, goal_y])
    
        # Estatísticas
        self.objetivo_alcancado = False
        self.tempo_inicio = self.get_clock().now()
        
        self.get_logger().info("🤖 P3-AT Q-Learning Executor iniciado!")
        self.get_logger().info(f"🎯 Objetivo: ({self.XF_M[0]:.2f}, {self.XF_M[1]:.2f})")
        self.get_logger().info(f"⚙️  Parâmetros: d={self.d}, vmax={self.vmax}, rotmax={self.rotmax}")

    def odom_callback(self, msg):
        """Recebe odometria do P3-AT"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        orientation = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.current_theta = euler[2]
        
        self.odom_received = True
        
    def calc_estados_discretos(self, erro_absoluto: np.ndarray):
        #Calcula estados discretos a partir dos erros
        dist_alvo = np.sqrt(erro_absoluto[0]**2 + erro_absoluto[1]**2)
        
        if dist_alvo < self.dist_tolerancia:
            erros_norm = np.array([0, 0])
            self.objetivo_alcancado = True # Adicionando a flag de estado            
        else:
            erros_norm = np.array([erro_absoluto[0] / dist_alvo, erro_absoluto[1] / dist_alvo])
        
        # Discretização
        idx_x = np.sum(erros_norm[0] > self.faixas_x)
        idx_y = np.sum(erros_norm[1] > self.faixas_y)
        
        # Limita aos índices válidos
        # Verificar se precisa mesmo dessa parte ou se a anterior não é suficiente
        # Limita aos índices válidos [0, N-1]
        max_idx_x = self.num_estados_x - 1
        max_idx_y = self.num_estados_y - 1
        
        idx_x = min(max(idx_x, 0), max_idx_x)
        idx_y = min(max(idx_y, 0), max_idx_y)
        
        
        estado_discreto = np.array([idx_x, idx_y])
        
        return estado_discreto, erros_norm, dist_alvo
        
    def escolher_melhor_acao(self, estado_discreto: np.ndarray) -> int:
        #Escolhe a melhor ação da Q-table (sem exploração)
        idx_x, idx_y = estado_discreto[0], estado_discreto[1]
        acao = np.argmax(self.Q_table[idx_x, idx_y, :])
        return acao
                        
    def mapear_acao_para_velocidades(self, acao_idx: int, erros_norm: np.ndarray):
        #Mapeia ação para velocidades do P3-AT
        idx_v = acao_idx // self.num_acoes_rot
        idx_rot = acao_idx % self.num_acoes_rot
        
        v_base = self.v_valores[idx_v]
        rot_base = self.rot_valores[idx_rot]

        v = v_base * erros_norm[0]
        rot = rot_base * erros_norm[1]
        
        # --- CLAMPING DE SEGURANÇA ---
        v = np.clip(v, -self.vmax, self.vmax)
        rot = np.clip(rot, -self.rotmax, self.rotmax)
        
        return v, rot

    def publish_velocities(self, v: float, rot: float):
        #Publica comandos de velocidade. Está estranho... Verificar...
        msg = Twist()
        msg.linear.x = float(v)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(rot)
        
        self.cmd_vel_pub_.publish(msg)

    def calcular_erro_posicao(self) -> np.ndarray:
        #Calcula erro de posição baseado na odometria
        # Erro de posição considerando d à frente
        erro_x = self.XF_M[0] - self.current_x
        erro_y = self.XF_M[1] - self.current_y
        
        # Matriz de rotação inversa
        cs = np.cos(self.current_theta)
        ss = np.sin(self.current_theta)
        
        erro_x_movel = cs*erro_x + ss*erro_y -self.d
        erro_y_movel = -ss*erro_x +cs*erro_y       
        
        return np.array([erro_x_movel, erro_y_movel])

    def control_loop(self):
        #Loop principal de execução da política
        if not self.odom_received:
            self.get_logger().info('Esperando dados de odometria...', once=True)
            return
 
     	# 1. Checa se já terminamos
        if self.objetivo_alcancado:
            self.parar_robo()
            return
               
        # Calcula erro de posição
        Er = self.calcular_erro_posicao()
        
        # Calcula estado discreto
        estado_discreto, erros_norm, dist_alvo = self.calc_estados_discretos(Er)
        
        # 3. VERIFICAÇÃO SIMPLIFICADA E UNIFICADA
        if self.objetivo_alcancado:
            # Lógica de log e cálculo de tempo só precisa ser feita UMA VEZ
            if not hasattr(self, '_log_tempo_finalizado'):
                tempo_decorrido = (self.get_clock().now() - self.tempo_inicio).nanoseconds / 1e9
                self.get_logger().info(f"🎯 OBJETIVO ALCANÇADO!")
                self.get_logger().info(f"⏱️  Tempo: {tempo_decorrido:.2f}s")
                self.get_logger().info(f"📍 Posição final: ({self.current_x:.3f}, {self.current_y:.3f})")
                self._log_tempo_finalizado = True # Evita log duplicado
                
            self.parar_robo()
            return
        
        # Escolhe a melhor ação
        acao = self.escolher_melhor_acao(estado_discreto)
        
        # Mapeia para velocidades
        v, rot = self.mapear_acao_para_velocidades(acao, erros_norm)
        
        # Publica comandos
        self.publish_velocities(v, rot)
        
        # Log periódico (a cada 2 segundos)
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % 20 == 0:
            self.get_logger().info(
                f"📊 Pos: ({self.current_x:.2f}, {self.current_y:.2f}), "
                f"Dist: {dist_alvo:.2f}m, Estado: ({estado_discreto[0]}, {estado_discreto[1]}), "
                f"v={v:.2f}, ω={rot:.2f}"
            )

    def parar_robo(self):
        #Parar o P3-AT (caso tenha sido forçada a parada da execução usando, por exemplo cntr + c)
        self.publish_velocities(0.0, 0.0)
        self.get_logger().info("🛑 P3-AT parado")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = P3ATQPolicy()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️  Interrompido pelo usuário")
        node.parar_robo()
    except Exception as e:
        print(f"❌ Erro: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

