import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random

class QNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(QNetwork, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim)
        )

    def forward(self, x):
        return self.fc(x)


class ReplayBuffer:
    def __init__(self, capacity=10000):
        self.buffer = []
        self.capacity = capacity
        self.pos = 0

    def push(self, s, a, r, s_next, done):
        data = (s, a, r, s_next, done)
        if len(self.buffer) < self.capacity:
            self.buffer.append(data)
        else:
            self.buffer[self.pos] = data
        self.pos = (self.pos + 1) % self.capacity

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        s, a, r, s_next, done = map(np.array, zip(*batch))
        return s, a, r, s_next, done

    def __len__(self):
        return len(self.buffer)


class DQNAgent:
    def __init__(self, state_dim, action_dim, lr=1e-3, gamma=0.99):
        self.q_net = QNetwork(state_dim, action_dim)
        self.target_net = QNetwork(state_dim, action_dim)
        self.target_net.load_state_dict(self.q_net.state_dict())
        self.optimizer = optim.Adam(self.q_net.parameters(), lr=lr)
        self.gamma = gamma
        self.replay = ReplayBuffer()
        self.action_dim = action_dim

    def select_action(self, state, epsilon):
        if random.random() < epsilon:
            return random.randint(0, self.action_dim - 1)
        state = torch.FloatTensor(state).unsqueeze(0)
        q_values = self.q_net(state)
        return q_values.argmax().item()

    def train_step(self, batch_size):
        if len(self.replay) < batch_size:
            return
        
        s, a, r, s_next, done = self.replay.sample(batch_size)
        s = torch.FloatTensor(s)
        a = torch.LongTensor(a)
        r = torch.FloatTensor(r)
        s_next = torch.FloatTensor(s_next)
        done = torch.FloatTensor(done)

        q_values = self.q_net(s).gather(1, a.unsqueeze(1)).squeeze(1)
        next_q = self.target_net(s_next).max(1)[0]
        target = r + self.gamma * next_q * (1 - done)

        loss = nn.MSELoss()(q_values, target.detach())

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    def update_target(self):
        self.target_net.load_state_dict(self.q_net.state_dict())
