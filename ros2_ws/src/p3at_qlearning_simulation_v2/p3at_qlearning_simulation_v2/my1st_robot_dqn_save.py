# my1st_robot_dqn.py
import numpy as np
import random
import os
import json
import matplotlib.pyplot as plt
from typing import Tuple, List, Dict, Any
import torch
import torch.nn as nn
import torch.optim as optim

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class ReplayBuffer:
    def __init__(self, capacity: int):
        self.capacity = capacity
        self.buffer = []
        self.pos = 0

    def push(self, state, action, reward, next_state, done):
        if len(self.buffer) < self.capacity:
            self.buffer.append(None)
        self.buffer[self.pos] = (state, action, reward, next_state, done)
        self.pos = (self.pos + 1) % self.capacity

    def sample(self, batch_size: int):
        batch = random.sample(self.buffer, batch_size)
        states, actions, rewards, next_states, dones = map(np.stack, zip(*batch))
        return states, actions, rewards, next_states, dones

    def __len__(self):
        return len(self.buffer)

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

class PolicyManager:
    def __init__(self):
        print("📁 Gerenciador de política DQN inicializado")

    def salvar_modelo(self, model: torch.nn.Module, parametros: Dict[str,Any], arquivo_base: str = "dqn_politica"):
        arquivo_pt = f"{arquivo_base}.pt"
        torch.save(model.state_dict(), arquivo_pt)
        arquivo_meta = f"{arquivo_base}_meta.npz"
        np.savez_compressed(arquivo_meta, parametros_json=json.dumps(parametros))
        print(f"✅ Modelo salvo: {arquivo_pt}")
        print(f"✅ Metadados salvos: {arquivo_meta}")
        return [arquivo_pt, arquivo_meta]

class RobotDQNAgent:
    def __init__(self, caso: int = 2):
        self.d = 0.5
        self.vmax = 2.0
        self.rotmax = 1.0
        self.h = 0.01
        self.caso = caso

        self.gamma = 0.99
        self.lr = 1e-3
        self.batch_size = 64
        self.buffer_capacity = 100000
        self.min_replay_size = 1000
        self.target_update_freq = 1000
        self.max_steps = 200

        self.epsilon = 1.0
        self.epsilon_min = 0.05
        self.epsilon_decay = 0.9995

        self.v_valores = np.array([0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.75, 1.0]) * self.vmax
        self.rot_valores = np.array([0, 0.05, 0.1, 0.2, 0.5, 1.0]) * self.rotmax
        self.num_acoes_v = len(self.v_valores)
        self.num_acoes_rot = len(self.rot_valores)
        self.num_acoes = self.num_acoes_v * self.num_acoes_rot

        self.state_dim = 3
        self.action_dim = self.num_acoes

        self.policy_net = DQN(self.state_dim, self.action_dim).to(DEVICE)
        self.target_net = DQN(self.state_dim, self.action_dim).to(DEVICE)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=self.lr)

        self.replay = ReplayBuffer(self.buffer_capacity)
        self.total_steps = 0

    def calcular_erro_inicial(self, destino: np.ndarray, pose: np.ndarray) -> np.ndarray:
        E = destino[:2] - pose[:2]
        cs = np.cos(pose[2])
        ss = np.sin(pose[2])
        Rot = np.array([[cs, ss], [-ss, cs]])
        Er_xy = Rot @ E - np.array([self.d, 0])
        Er = np.array([Er_xy[0], Er_xy[1], pose[2]])
        return Er

    def calc_estados_continuos(self, erro_bruto: np.ndarray) -> Tuple[np.ndarray, float]:
        dist_alvo = np.sqrt(erro_bruto[0]**2 + erro_bruto[1]**2)
        if dist_alvo < 0.01:
            erros_norm = np.array([0.0, 0.0])
        else:
            erros_norm = np.array([erro_bruto[0] / dist_alvo, erro_bruto[1] / dist_alvo])
        state = np.array([erros_norm[0], erros_norm[1], dist_alvo], dtype=np.float32)
        return state, dist_alvo

    def simular_acao_caso1(self, Er: np.ndarray, U: np.ndarray) -> np.ndarray:
        B = np.array([[-1, Er[1]], [0, -(Er[0] + self.d)], [0, 1]])
        Er_pt = B @ U
        Er_novo = Er.copy()
        for i in range(100):
            Er_novo[0] = Er_novo[0] + self.h * Er_pt[0]
            Er_novo[1] = Er_novo[1] + self.h * Er_pt[1]
            Er_novo[2] = (Er_novo[2] + self.h * Er_pt[2]) % (2 * np.pi)
        return Er_novo

    def simular_acao_caso2(self, pose: np.ndarray, destino: np.ndarray, U: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        pose_novo = pose.copy()
        for i in range(100):
            pose_novo[0] = pose_novo[0] + self.h * U[0] * np.cos(pose_novo[2])
            pose_novo[1] = pose_novo[1] + self.h * U[0] * np.sin(pose_novo[2])
            pose_novo[2] = (pose_novo[2] + self.h * U[1]) % (2 * np.pi)
        Er_novo = self.calcular_erro_inicial(destino, pose_novo)
        return Er_novo, pose_novo

    def calcular_recompensa(self, state: np.ndarray, next_state: np.ndarray) -> float:
        recompensa = -1.0
        if next_state[2] < 0.2:
            recompensa = 100.0
        if next_state[2] > state[2]:
            recompensa = -5.0
        return recompensa

    def map_action(self, action_idx: int, state: np.ndarray) -> Tuple[float, float]:
        idx_v = action_idx // self.num_acoes_rot
        idx_rot = action_idx % self.num_acoes_rot
        erros_dir = np.array([state[0], state[1]])
        v = self.v_valores[idx_v] * erros_dir[0]
        rot = self.rot_valores[idx_rot] * erros_dir[1]
        return float(v), float(rot)

    def select_action(self, state: np.ndarray) -> int:
        if random.random() < self.epsilon:
            return random.randrange(self.action_dim)
        state_t = torch.FloatTensor(state).unsqueeze(0).to(DEVICE)
        with torch.no_grad():
            qvals = self.policy_net(state_t)
        return int(qvals.argmax(dim=1).item())

    def update(self):
        if len(self.replay) < self.min_replay_size:
            return
        states, actions, rewards, next_states, dones = self.replay.sample(self.batch_size)
        states_t = torch.FloatTensor(states).to(DEVICE)
        next_states_t = torch.FloatTensor(next_states).to(DEVICE)
        actions_t = torch.LongTensor(actions.reshape(-1,1)).to(DEVICE)
        rewards_t = torch.FloatTensor(rewards.reshape(-1,1)).to(DEVICE)
        dones_t = torch.FloatTensor(dones.reshape(-1,1)).to(DEVICE)

        q_values = self.policy_net(states_t).gather(1, actions_t)
        next_q = self.target_net(next_states_t).max(1)[0].detach().unsqueeze(1)
        expected_q = rewards_t + (1 - dones_t) * self.gamma * next_q

        loss = nn.MSELoss()(q_values, expected_q)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    def treinar(self, num_episodios: int = 5000, passos_por_episodio: int = 200) -> List[Tuple[int,int]]:
        historico = []
        for ep in range(1, num_episodios + 1):
            destino = np.random.uniform(-10, 10, 2)
            pose = np.array([0.0, 0.0, 0.0])
            Er = self.calcular_erro_inicial(destino, pose)
            state, dist = self.calc_estados_continuos(Er)

            for t in range(1, passos_por_episodio + 1):
                action = self.select_action(state)
                v, rot = self.map_action(action, state)
                U = np.array([v, rot])

                if self.caso == 1:
                    Er_novo = self.simular_acao_caso1(Er, U)
                else:
                    Er_novo, pose = self.simular_acao_caso2(pose, destino, U)

                next_state, dist_next = self.calc_estados_continuos(Er_novo)
                reward = self.calcular_recompensa(state, next_state)
                done = (reward > 0)

                self.replay.push(state.astype(np.float32), np.array(action, dtype=np.int64), np.array(reward, dtype=np.float32), next_state.astype(np.float32), np.array(done, dtype=np.float32))

                self.update()

                Er = Er_novo
                state = next_state
                self.total_steps += 1

                if self.total_steps % self.target_update_freq == 0:
                    self.target_net.load_state_dict(self.policy_net.state_dict())

                if done:
                    break

            self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
            historico.append((ep, t))

            if ep % 100 == 0:
                print(f"Episódio {ep}, Passos: {t}, Epsilon: {self.epsilon:.4f}, ReplaySize: {len(self.replay)}")

        return historico

    def testar_politica(self, model_path: str = None, max_passos: int = 100) -> Tuple[List, bool]:
        if model_path:
            self.policy_net.load_state_dict(torch.load(model_path, map_location=DEVICE))
            self.policy_net.to(DEVICE)
        destino = np.array([4.0, 3.0])
        pose = np.array([0.0, 0.0, 0.0])
        Er = self.calcular_erro_inicial(destino, pose)
        trajetoria = []
        for passo in range(max_passos):
            state, dist = self.calc_estados_continuos(Er)
            trajetoria.append(state.copy())
            if state[2] < 0.2:
                return trajetoria, True
            action = self.select_action(state)
            v, rot = self.map_action(action, state)
            U = np.array([v, rot])
            if self.caso == 1:
                Er = self.simular_acao_caso1(Er, U)
            else:
                Er, pose = self.simular_acao_caso2(pose, destino, U)
        return trajetoria, False

    def plotar_progresso(self, historico: List[Tuple[int, int]]):
        episodios = [h[0] for h in historico]
        passos = [h[1] for h in historico]
        plt.figure(figsize=(12,4))
        plt.plot(episodios, passos, alpha=0.6)
        plt.xlabel('Episódio')
        plt.ylabel('Número de Passos')
        plt.title('Progresso do Treinamento (DQN)')
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    print("(Pytorch) Modo de execução do código selecionado:", DEVICE)

    agent = RobotDQNAgent(caso=1)
    print("Iniciando treinamento DQN...")
    historico = agent.treinar(num_episodios=2000, passos_por_episodio=agent.max_steps)

    manager = PolicyManager()
    parametros = {
        'd': agent.d,
        'vmax': agent.vmax,
        'rotmax': agent.rotmax,
        'caso': agent.caso,
        'gamma': agent.gamma,
        'lr': agent.lr,
        'state_dim': agent.state_dim,
        'action_dim': agent.action_dim,
        'v_valores': agent.v_valores.tolist(),
        'rot_valores': agent.rot_valores.tolist()
    }
    arquivos = manager.salvar_modelo(agent.policy_net, parametros, "dqn_politica")

    trajetoria, sucesso = agent.testar_politica("dqn_politica.pt")
    print(f"Teste: {'Sucesso' if sucesso else 'Falhou'} em {len(trajetoria)} passos")

    agent.plotar_progresso(historico)
