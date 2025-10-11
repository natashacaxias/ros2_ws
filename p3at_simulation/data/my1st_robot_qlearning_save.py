import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List, Dict, Any
import random
import os
import json # Importa a biblioteca para salvar parâmetros em JSON dentro do arquivo .npz

class TabelaQManager:
    def __init__(self):
        self.formatos_suportados = ['.dat', '.npy', '.npz'] 
        print("📁 Gerenciador de Tabela Q inicializado")
        print(f"Formatos suportados: {', '.join(self.formatos_suportados)}")
        
    def salvar_tabela_completa(self, robot, nome_base: str = "tabela_q_treinada"):
        """
        Salva a tabela Q e todos os parâmetros em múltiplos formatos
        
        Args:
            robot: Instância do RobotQLearningV2 treinado
            nome_base: Nome base dos arquivos (sem extensão)
        """
        print(f"\n💾 SALVANDO TABELA Q TREINADA: {nome_base}")
        print("=" * 50)

        # Coleta todos os parâmetros importantes
        dados_completos = self._extrair_dados_robot(robot)
        
        # Salva em cada formato
        arquivos_gerados = []

        # 1. Formato NumPy binário (.npy) e pacote compactado (.npz)
        # O npz é usado para salvar múltiplos arrays, incluindo a tabela Q e os parâmetros
        arquivo_npz = self.salvar_numpy(dados_completos, f"{nome_base}.npz")
        arquivos_gerados.append(arquivo_npz)

        # 2. Texto estruturado (.dat) - compatível com Scilab
        arquivo_dat = self.salvar_texto_scilab(dados_completos, f"{nome_base}.dat")
        arquivos_gerados.append(arquivo_dat)

        print(f"\n✅ SALVAMENTO CONCLUÍDO!")
        print(f"Arquivos gerados: {len(arquivos_gerados)}")
        for arquivo in arquivos_gerados:
            tamanho = os.path.getsize(arquivo) if os.path.exists(arquivo) else 0
            print(f"  📄 {arquivo} ({tamanho/1024:.1f} KB)")
        
        return arquivos_gerados

    def _extrair_dados_robot(self, robot) -> Dict[str, Any]:
        """Extrai todos os dados importantes do robô"""
        return {
            'Q_table': robot.Q_table,
            'parametros': {
                'd': robot.d,
                'vmax': robot.vmax,
                'rotmax': robot.rotmax,
                'h': robot.h,
                'caso': robot.caso,
                'alfa': robot.alfa,
                'gama': robot.gama,
                'epsilon_final': robot.epsilon,
                'num_episodios': robot.num_episodios,
                'passos_por_episodio': robot.passos_por_episodio,
                'dist_tolerancia': robot.dist_tolerancia
            },
            'discretizacao': {
                'faixas_x': robot.faixas_x,
                'faixas_y': robot.faixas_y,
                'centro_x': robot.centro_x,
                'centro_y': robot.centro_y,
                'num_estados_x': robot.num_estados_x,
                'num_estados_y': robot.num_estados_y
            },
            'acoes': {
                'v_valores': robot.v_valores,
                'rot_valores': robot.rot_valores,
                'num_acoes_v': robot.num_acoes_v,
                'num_acoes_rot': robot.num_acoes_rot,
                'num_acoes': robot.num_acoes
            }
        }

    def salvar_numpy(self, dados: Dict, arquivo: str) -> str:
        """
        Salva a tabela Q e os parâmetros no formato .npz (NumPy compactado)
        para fácil carregamento em outro script Python.
        """
        Q_table = dados['Q_table']
        
        # Converte os parâmetros (que devem ser tipos nativos) para JSON
        # Se algum parâmetro ainda for numpy.float64, também precisaria ser convertido:
        # por exemplo, dados['parametros']['alfa'] = float(dados['parametros']['alfa'])
        parametros_json = json.dumps(dados['parametros'])
        
        # CORREÇÃO: Converter valores numéricos específicos do NumPy (como int64)
        # para tipos nativos do Python usando int() ou float().
        discretizacao_json = json.dumps({
            'faixas_x': dados['discretizacao']['faixas_x'].tolist(), 
            'faixas_y': dados['discretizacao']['faixas_y'].tolist(),
            # APLICAR INT() AQUI:
            'centro_x': int(dados['discretizacao']['centro_x']), 
            'centro_y': int(dados['discretizacao']['centro_y']),
            # APLICAR INT() AQUI:
            'num_estados_x': int(dados['discretizacao']['num_estados_x']),
            'num_estados_y': int(dados['discretizacao']['num_estados_y'])
        })
        
        acoes_json = json.dumps({
            'v_valores': dados['acoes']['v_valores'].tolist(), 
            'rot_valores': dados['acoes']['rot_valores'].tolist(),
            # APLICAR INT() AQUI:
            'num_acoes_v': int(dados['acoes']['num_acoes_v']),
            'num_acoes_rot': int(dados['acoes']['num_acoes_rot']),
            'num_acoes': int(dados['acoes']['num_acoes'])
        })
        
        np.savez_compressed(
            arquivo, 
            Q_table=Q_table, 
            parametros_json=parametros_json,
            discretizacao_json=discretizacao_json,
            acoes_json=acoes_json
        )
        
        print(f"✅ NumPy NPZ: {arquivo} (Q_table e Parâmetros salvos)")
        return arquivo

    def salvar_texto_scilab(self, dados: Dict, arquivo: str) -> str:
        """
        Salva em formato texto estruturado compatível com Scilab
        Exatamente como seria criado no script original
        """
        Q_table = dados['Q_table']
        
        with open(arquivo, 'w') as f:
            # Cabeçalho informativo
            f.write("// Tabela Q treinada - Formato compatível com Scilab\n")
            f.write("// Gerado automaticamente pelo Python\n")
            f.write(f"// Dimensões: {Q_table.shape[0]} x {Q_table.shape[1]} x {Q_table.shape[2]}\n")
            f.write("//\n")
            
            # Parâmetros importantes
            f.write("// PARÂMETROS DO TREINAMENTO:\n")
            for chave, valor in dados['parametros'].items():
                f.write(f"// {chave} = {valor}\n")
            f.write("//\n")
            
            # Discretização
            f.write("// DISCRETIZAÇÃO:\n")
            f.write(f"// faixas_x = {dados['discretizacao']['faixas_x'].tolist()}\n")
            f.write(f"// faixas_y = {dados['discretizacao']['faixas_y'].tolist()}\n")
            f.write("//\n")
            
            # Início dos dados da tabela Q
            f.write("// TABELA Q (formato: Q_table(x, y, acao) = valor):\n")
            f.write("Q_table = zeros(%d, %d, %d);\n\n" % Q_table.shape)
            
            # Salva apenas valores não-zero para eficiência
            valores_nao_zero = 0
            for x in range(Q_table.shape[0]):
                for y in range(Q_table.shape[1]):
                    for a in range(Q_table.shape[2]):
                        valor = Q_table[x, y, a]
                        if abs(valor) > 1e-10:  # Evita valores muito pequenos
                            f.write(f"Q_table({x+1}, {y+1}, {a+1}) = {valor:.8f};\n")
                            valores_nao_zero += 1
            
            f.write(f"\n// Total de valores não-zero: {valores_nao_zero}\n")
            f.write(f"// Densidade da tabela: {valores_nao_zero/Q_table.size*100:.2f}%\n")
        
        print(f"✅ Texto Scilab: {arquivo} ({valores_nao_zero} valores)")
        return arquivo


class RobotQLearningV2:
    def __init__(self, caso: int = 2):
        # Características do robô
        self.d = 0.5  # distância à frente do robô que deverá ser posicionado em metros
        self.vmax = 3  # velocidade máxima de translação em metros por segundo
        self.rotmax = 1 # velocidade máxima de rotação em radianos por segundo
        
        # Parâmetros da simulação
        self.h = 0.01  # passo de integração
        self.caso = caso  # 1: atualiza erro direto, 2: atualiza pose e recalcula erro
        
        # Hiperparâmetros do Q-Learning
        self.alfa = 0.1  # Taxa de aprendizado
        self.gama = 0.9  # Fator de desconto
        self.epsilon_inicial = 1.0
        self.epsilon_min = 0.05
        self.taxa_decaimento = 0.999
        self.num_episodios = 5000
        self.passos_por_episodio = 200
        self.dist_tolerancia = 0.2
        
        # Definição das faixas de discretização do erro normalizado
        self.faixas_x = np.array([-1, -0.5, -0.25, -0.125, -0.0625, 0, 
                                  0.0625, 0.125, 0.25, 0.5, 1])
        self.faixas_y = np.array([-1, -0.5, -0.25, -0.125, -0.0625, 0, 
                                  0.0625, 0.125, 0.25, 0.5, 1])
        
        # Encontra os índices do centro (erro zero)
        self.centro_x = np.where(self.faixas_x == 0)[0][0] + 1  # +1 para indexação 1-based
        self.centro_y = np.where(self.faixas_y == 0)[0][0] + 1
        
        # Conjunto de ações (velocidades discretas) - apenas valores positivos
        self.v_valores = np.array([0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.75, 1.0]) * self.vmax
        self.rot_valores = np.array([0, 0.05, 0.1, 0.2, 0.5, 1.0]) * self.rotmax
        
        self.num_acoes_v = len(self.v_valores)
        self.num_acoes_rot = len(self.rot_valores)
        self.num_acoes = self.num_acoes_v * self.num_acoes_rot
        
        # Tamanho da tabela Q
        self.num_estados_x = len(self.faixas_x)
        self.num_estados_y = len(self.faixas_y)
        
        # Inicialização da tabela Q com zeros
        self.Q_table = np.zeros((self.num_estados_x, self.num_estados_y, self.num_acoes))
        
        self.epsilon = self.epsilon_inicial
    
    def calc_estados_discretos(self, erro_bruto: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Calcula os estados discretos a partir dos erros brutos
        
        Args:
            erro_bruto: Vetor de erros [x, y, theta]
            
        Returns:
            estado_discreto: Índices discretos [idx_x, idx_y]
            erros_norm: Erros normalizados [x_norm, y_norm]
            dist_alvo: Distância ao alvo
        """
        # Distância ao objetivo
        dist_alvo = np.sqrt(erro_bruto[0]**2 + erro_bruto[1]**2)
        
        # Evita divisão por zero
        if dist_alvo < 0.01:
            erros_norm = np.array([0, 0])
        else:
            erros_norm = np.array([erro_bruto[0] / dist_alvo, erro_bruto[1] / dist_alvo])
        
        # Discretização dos erros normalizados
        idx_x = np.sum(erros_norm[0] > self.faixas_x) + 1
        idx_y = np.sum(erros_norm[1] > self.faixas_y) + 1
        
        # Garante que os índices estejam dentro dos limites
        idx_x = max(1, min(idx_x, self.num_estados_x))
        idx_y = max(1, min(idx_y, self.num_estados_y))
        
        estado_discreto = np.array([idx_x, idx_y])
        
        return estado_discreto, erros_norm, dist_alvo
    
    def escolher_acao(self, estado_discreto: np.ndarray) -> int:
        """
        Escolhe uma ação usando estratégia epsilon-greedy
        """
        if random.random() < self.epsilon:
            # Exploração: escolhe uma ação aleatória
            acao = round(random.random() * (self.num_acoes - 1)) + 1
        else:
            # Explotação: escolhe a melhor ação da tabela Q
            idx_x, idx_y = estado_discreto[0], estado_discreto[1]
            acao = np.argmax(self.Q_table[idx_x - 1, idx_y - 1, :]) + 1  # Ajusta para 1-based
        
        return acao
    
    def mapear_acao_para_velocidades(self, acao: int, erros_norm: np.ndarray) -> Tuple[float, float]:
        """
        Mapeia a ação escolhida para as velocidades (v e rot)
        """
        idx_v = (acao - 1) // self.num_acoes_rot
        idx_rot = (acao - 1) % self.num_acoes_rot
        
        v = self.v_valores[idx_v] * erros_norm[0]
        rot = self.rot_valores[idx_rot] * erros_norm[1]
        
        return v, rot
    
    def calcular_erro_inicial(self, destino: np.ndarray, pose: np.ndarray) -> np.ndarray:
        """
        Calcula o erro inicial no frame do robô
        
        Args:
            destino: Posição final do ponto d à frente do robô [x, y]
            pose: Pose inicial do robô [x, y, theta]
            
        Returns:
            Er: Erro no frame do robô [x, y, theta]
        """
        # Erro de posicionamento no sistema fixo
        E = destino[:2] - pose[:2]
        
        # Matriz de rotação inversa para alinhar o sistema fixo ao do robô
        cs = np.cos(pose[2])
        ss = np.sin(pose[2])
        Rot = np.array([[cs, ss], [-ss, cs]])
        
        # Erro no frame do robô
        Er_xy = Rot @ E - np.array([self.d, 0])
        Er = np.array([Er_xy[0], Er_xy[1], pose[2]])
        
        return Er
    
    def simular_acao_caso1(self, Er: np.ndarray, U: np.ndarray) -> np.ndarray:
        """
        Caso 1: Calcula a velocidade do erro no frame do robô
        """
        # Modelo cinemático (equação 39 do PDF)
        B = np.array([[-1, Er[1]], 
                      [0, -(Er[0] + self.d)], 
                      [0, 1]])
        
        Er_pt = B @ U
        Er_novo = Er.copy()
        
        # Integração numérica
        for i in range(10): #alterei de 100 mili para 10 ass:ab
            Er_novo[0] = Er_novo[0] + self.h * Er_pt[0]
            Er_novo[1] = Er_novo[1] + self.h * Er_pt[1]
            Er_novo[2] = (Er_novo[2] + self.h * Er_pt[2]) % (2 * np.pi)
        
        return Er_novo
    
    def simular_acao_caso2(self, pose: np.ndarray, destino: np.ndarray, U: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Caso 2: Atualiza a pose do robô e recalcula os erros
        """
        pose_novo = pose.copy()
        
        # Integração da pose do robô
        for i in range(10): 
            pose_novo[0] = pose_novo[0] + self.h * U[0] * np.cos(pose_novo[2])
            pose_novo[1] = pose_novo[1] + self.h * U[0] * np.sin(pose_novo[2])
            pose_novo[2] = (pose_novo[2] + self.h * U[1]) % (2 * np.pi)
        
        # Recalcula o erro no novo estado
        Er_novo = self.cacular_erro_inicial(destino, pose_novo)
        
        return Er_novo, pose_novo
    
    def calcular_recompensa(self, estado_discreto: np.ndarray, estado_proximo_discreto: np.ndarray) -> float:
        """
        Calcula a recompensa baseada no estado atual e próximo
        """
        # Penalidade de tempo
        recompensa = -1
        
        # Grande recompensa por chegar ao centro (alvo)
        if (estado_discreto[0] == self.centro_x and estado_discreto[1] == self.centro_y):
            recompensa = 100
        
        # Penalidade por se afastar do alvo
        if np.sum(estado_proximo_discreto) > np.sum(estado_discreto):
            recompensa = -5
        
        return recompensa
    
    def atualizar_tabela_q(self, estado_discreto: np.ndarray, acao: int, 
                          recompensa: float, estado_proximo_discreto: np.ndarray):
        """
        Atualiza a tabela Q usando a equação de Bellman
        """
        # Índices (ajustados para 0-based do Python)
        idx_x, idx_y = estado_discreto[0] - 1, estado_discreto[1] - 1
        idx_x_prox, idx_y_prox = estado_proximo_discreto[0] - 1, estado_proximo_discreto[1] - 1
        acao_idx = acao - 1
        
        Q_s_a = self.Q_table[idx_x, idx_y, acao_idx]
        Q_max_proximo = np.max(self.Q_table[idx_x_prox, idx_y_prox, :])
        
        self.Q_table[idx_x, idx_y, acao_idx] = Q_s_a + self.alfa * (
            recompensa + self.gama * Q_max_proximo - Q_s_a
        )
    
    def treinar(self) -> List[Tuple[int, int]]:
        """
        Executa o treinamento do Q-Learning
        """
        historico = []
        
        for i in range(1, self.num_episodios + 1):
            # Preparação do episódio
            destino = np.random.uniform(-10, 10, 2)  # Posição final do ponto d à frente do robô
            pose = np.array([0, 0, 0])  # Pose inicial do robô
            
            # Calcula o erro inicial
            Er = self.calcular_erro_inicial(destino, pose)
            
            # Calcula o estado inicial
            estado_discreto, erros_norm, dist_alvo = self.calc_estados_discretos(Er)
            
            for j in range(1, self.passos_por_episodio + 1):
                # Escolhe a ação
                acao = self.escolher_acao(estado_discreto)
                
                # Mapeia a ação para velocidades
                v, rot = self.mapear_acao_para_velocidades(acao, erros_norm)
                U = np.array([v, rot])
                
                # Aplica a ação (simulação)
                if self.caso == 1:
                    Er_novo = self.simular_acao_caso1(Er, U)
                else:  # caso == 2
                    Er_novo, pose = self.simular_acao_caso2(pose, destino, U)
                
                # Calcula o próximo estado
                estado_proximo_discreto, erros_norm, dist_alvo = self.calc_estados_discretos(Er_novo)
                
                # Calcula a recompensa
                recompensa = self.calcular_recompensa(estado_discreto, estado_proximo_discreto)
                
                # Atualiza a tabela Q
                self.atualizar_tabela_q(estado_discreto, acao, recompensa, estado_proximo_discreto)
                
                # Atualiza o estado
                Er = Er_novo
                estado_discreto = estado_proximo_discreto
                
                # Verifica se o episódio terminou
                if recompensa > 0 or j == self.passos_por_episodio:
                    break
            
            # Registra o progresso
            historico.append((i, j))
            
            # Imprime o progresso a cada 100 episódios
            if i % 100 == 0:
                print(f"Episódio {i}, Passos: {j}, Epsilon: {self.epsilon:.4f}")
            
            # Decaimento do epsilon
            self.epsilon = max(self.epsilon_min, self.epsilon * self.taxa_decaimento)
        
        return historico
    
    def plotar_progresso(self, historico: List[Tuple[int, int]]):
        """
        Plota o progresso do treinamento
        """
        episodios = [h[0] for h in historico]
        passos = [h[1] for h in historico]
        
        plt.figure(figsize=(15, 5))
        
        # Gráfico de passos por episódio
        plt.subplot(1, 3, 1)
        plt.plot(episodios, passos, alpha=0.6)
        plt.xlabel('Episódio')
        plt.ylabel('Número de Passos')
        plt.title('Progresso do Treinamento')
        plt.grid(True)
        
        # Média móvel
        window = 100
        if len(passos) >= window:
            media_movel = np.convolve(passos, np.ones(window)/window, mode='valid')
            plt.subplot(1, 3, 2)
            plt.plot(range(window-1, len(passos)), media_movel)
            plt.xlabel('Episódio')
            plt.ylabel('Média de Passos (100 episódios)')
            plt.title('Tendência de Aprendizado')
            plt.grid(True)
        
        # Visualização da tabela Q (valor máximo por estado)
        plt.subplot(1, 3, 3)
        Q_max = np.max(self.Q_table, axis=2)
        im = plt.imshow(Q_max, cmap='viridis', origin='lower')
        plt.colorbar(im)
        plt.xlabel('Estado Y')
        plt.ylabel('Estado X')
        plt.title('Valores Q Máximos por Estado')
        
        # Marca o centro (objetivo)
        plt.plot(self.centro_y-1, self.centro_x-1, 'r*', markersize=15, label='Objetivo')
        plt.legend()
        
        plt.tight_layout()
        plt.show()
    
    def testar_politica(self, max_passos: int = 100) -> Tuple[List, bool]:
        """
        Testa a política aprendida sem exploração
        
        Returns:
            trajetoria: Lista de estados visitados
            sucesso: Se chegou ao objetivo
        """
        # Configuração inicial
        destino = np.array([4, 3])
        pose = np.array([0, 0, 0])
        Er = self.calcular_erro_inicial(destino, pose)
        
        trajetoria = []
        epsilon_original = self.epsilon
        self.epsilon = 0  # Sem exploração
        
        for passo in range(max_passos):
            estado_discreto, erros_norm, dist_alvo = self.calc_estados_discretos(Er)
            trajetoria.append(estado_discreto.copy())
            
            # Verifica se chegou ao objetivo
            if (estado_discreto[0] == self.centro_x and estado_discreto[1] == self.centro_y):
                self.epsilon = epsilon_original
                return trajetoria, True
            
            # Escolhe ação e executa
            acao = self.escolher_acao(estado_discreto)
            v, rot = self.mapear_acao_para_velocidades(acao, erros_norm)
            U = np.array([v, rot])
            
            if self.caso == 1:
                Er = self.simular_acao_caso1(Er, U)
            else:
                Er, pose = self.simular_acao_caso2(pose, destino, U)
        
        self.epsilon = epsilon_original
        return trajetoria, False

# Exemplo de uso
if __name__ == "__main__":
    # Cria e treina o robô (Caso 1: para testar)
    robot = RobotQLearningV2(caso=1)
    print("Iniciando treinamento...")
    print(f"Configuração: Caso {robot.caso}")
    print(f"Parâmetros: d={robot.d}m, vmax={robot.vmax}m/s, rotmax={robot.rotmax:.4f}rad/s")
    print(f"Discretização: {robot.num_estados_x}x{robot.num_estados_y} estados, {robot.num_acoes} ações")
    print(f"Centro (objetivo): ({robot.centro_x}, {robot.centro_y})")
    
    historico = robot.treinar()

    # 2. Salvar em formato Scilab
    manager = TabelaQManager()
    manager.salvar_tabela_completa(robot, "meu_robo_treinado")
    
    print(f"\nTreinamento concluído!")
    print(f"Total de episódios: {len(historico)}")
    print(f"Epsilon final: {robot.epsilon:.4f}")
    
    # Testa a política aprendida
    print("\nTestando política aprendida...")
    trajetoria, sucesso = robot.testar_politica()
    print(f"Teste: {'Sucesso' if sucesso else 'Falhou'} em {len(trajetoria)} passos")
    
    # Plota os resultados
    robot.plotar_progresso(historico)
