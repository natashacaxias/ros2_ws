#!/usr/bin/env python3
"""
Treinamento offline DQN para robô P3AT (compatível com ROS2 controller).
Gera dqn_politica.pt e dqn_politica_meta.npz para uso no Gazebo.
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import random
import json
import os
from typing import Any, Dict, List, Tuple
import matplotlib.pyplot as plt

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


# ============================
#  REDE NEURAL DQN
# ============================
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


# ============================
#  REPLAY BUFFER
# ============================
class ReplayBuffer:
    def __init__(self, capacity: int = 100_000):
        self.capacity = capacity
        self.buffer = []
        self.pos = 0

    def push(self, s, a, r, ns, d):
        if len(self.buffer) < self.capacity:
            self.buffer.append(None)
        self.buffer[self.pos] = (s, a, r, ns, d)
        self.pos = (self.pos + 1) % self.capacity

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        s, a, r, ns, d = map(np.stack, zip(*batch))
        return s, a, r, ns, d

    def __len__(self):
        return len(self.buffer)


# ============================
#  AGENTE DQN
# ============================
class RobotDQNAgent:
    def __init__(self):
        # Configurações básicas
        self.d = 0.5
        self.vmax = 1.0
        self.rotmax = 1.0
        self.h = 0.01

        # DQN hyperparams
        self.gamma = 0.99
        self.lr = 1e-3
        self.batch_size = 64
        self.buffer = ReplayBuffer()
        self.min_replay_size = 1000
        self.target_update_freq = 500
        self.total_steps = 0

        # Epsilon-greedy
        self.epsilon = 1.0
        self.epsilon_min = 0.05
        self.epsilon_decay = 0.995

        # Ações discretas
        self.v_valores = np.array([0, 0.1, 0.2, 0.3, 0.5, 0.75, 1.0]) * self.vmax
        self.rot_valores = np.array([0, 0.1, 0.2, 0.3, 0.6, 1.0]) * self.rotmax
        self.state_dim = 3
        self.action_dim = len(self.v_valores) * len(self.rot_valores)

        # Modelos
        self.policy_net = DQN(self.state_dim, self.action_dim).to(DEVICE)
        self.target_net = DQN(self.state_dim, self.action_dim).to(DEVICE)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=self.lr)

    # ============================
    #  FUNÇÕES DE SIMULAÇÃO
    # ============================
    def calcular_erro(self, destino, pose):
        E = destino[:2] - pose[:2]
        cs, ss = np.cos(pose[2]), np.sin(pose[2])
        Rot = np.array([[cs, ss], [-ss, cs]])
        Er_xy = Rot @ E - np.array([self.d, 0])
        return np.array([Er_xy[0], Er_xy[1], pose[2]])

    def calc_estado(self, Er):
        dist = np.hypot(Er[0], Er[1])
        nx, ny = (0.0, 0.0) if dist < 1e-6 else (Er[0]/dist, Er[1]/dist)
        return np.array([nx, ny, dist], dtype=np.float32), dist

    def simular_passo(self, pose, U):
        pose_novo = pose.copy()
        for _ in range(50):
            pose_novo[0] += self.h * U[0] * np.cos(pose_novo[2])
            pose_novo[1] += self.h * U[0] * np.sin(pose_novo[2])
            pose_novo[2] = (pose_novo[2] + self.h * U[1]) % (2*np.pi)
        return pose_novo

    # ============================
    #  POLÍTICA DQN
    # ============================
    def map_action(self, idx, state):
        i_v = idx // len(self.rot_valores)
        i_r = idx % len(self.rot_valores)
        v = self.v_valores[i_v] * state[0]
        rot = self.rot_valores[i_r] * state[1]
        return float(v), float(rot)

    def select_action(self, state):
        if random.random() < self.epsilon:
            return random.randrange(self.action_dim)
        with torch.no_grad():
            q = self.policy_net(torch.FloatTensor(state).unsqueeze(0).to(DEVICE))
        return int(q.argmax(dim=1).item())

    def update(self):
        if len(self.buffer) < self.min_replay_size:
            return
        s, a, r, ns, d = self.buffer.sample(self.batch_size)
        s = torch.FloatTensor(s).to(DEVICE)
        ns = torch.FloatTensor(ns).to(DEVICE)
        a = torch.LongTensor(a).unsqueeze(1).to(DEVICE)
        r = torch.FloatTensor(r).unsqueeze(1).to(DEVICE)
        d = torch.FloatTensor(d).unsqueeze(1).to(DEVICE)

        qsa = self.policy_net(s).gather(1, a)
        with torch.no_grad():
            next_q = self.target_net(ns).max(1)[0].unsqueeze(1)
            target = r + (1 - d) * self.gamma * next_q

        loss = nn.MSELoss()(qsa, target)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    # ============================
    #  TREINAMENTO
    # ============================
    def treinar(self, episodios=2000, max_steps=200):
        hist = []
        for ep in range(1, episodios+1):
            destino = np.array([random.uniform(-5, 5), random.uniform(-5, 5)])
            pose = np.array([0.0, 0.0, 0.0])
            Er = self.calcular_erro(destino, pose)
            state, dist = self.calc_estado(Er)
            for t in range(max_steps):
                action = self.select_action(state)
                v, rot = self.map_action(action, state)
                pose = self.simular_passo(pose, np.array([v, rot]))
                Er = self.calcular_erro(destino, pose)
                next_state, next_dist = self.calc_estado(Er)
                reward = -next_dist
                done = next_dist < 0.2

                self.buffer.push(state, action, reward, next_state, done)
                self.update()

                state = next_state
                self.total_steps += 1
                if self.total_steps % self.target_update_freq == 0:
                    self.target_net.load_state_dict(self.policy_net.state_dict())

                if done:
                    break

            self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
            hist.append((ep, t))
            if ep % 100 == 0:
                print(f"Ep {ep:04d} | Steps={t:3d} | eps={self.epsilon:.3f} | Replay={len(self.buffer)}")

        return hist

    # ============================
    #  SALVAR MODELO
    # ============================
    def salvar_modelo(self, base="dqn_politica"):
        torch.save(self.policy_net.state_dict(), f"{base}.pt")
        meta = {
            'd': self.d,
            'v_valores': self.v_valores.tolist(),
            'rot_valores': self.rot_valores.tolist(),
            'state_dim': self.state_dim,
            'action_dim': self.action_dim
        }
        np.savez_compressed(f"{base}_meta.npz", parametros_json=json.dumps(meta))
        print(f"✅ Modelo salvo: {base}.pt / {base}_meta.npz")


# ============================
#  EXECUÇÃO PRINCIPAL
# ============================
if __name__ == "__main__":
    print("(PyTorch) Treinamento DQN P3AT iniciado em", DEVICE)
    agent = RobotDQNAgent()
    hist = agent.treinar(episodios=1000, max_steps=200)
    agent.salvar_modelo()

    plt.plot([h[0] for h in hist], [h[1] for h in hist])
    plt.xlabel("Episódios")
    plt.ylabel("Passos até o sucesso")
    plt.title("Progresso do Treinamento DQN")
    plt.grid(True)
    plt.show()
