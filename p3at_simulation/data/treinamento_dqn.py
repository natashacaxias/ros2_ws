from p3at_simulation.dqn_agent import DQNAgent
from p3at_simulation.env_ros2_gym import Ros2Env
import torch

env = Ros2Env()
agent = DQNAgent(state_dim=6, action_dim=6)
episodes = 500
batch_size = 64
epsilon = 1.0
epsilon_decay = 0.995
epsilon_min = 0.1

for ep in range(episodes):
    state = env.reset()
    total_reward = 0
    for t in range(300):
        action_idx = agent.select_action(state, epsilon)
        v = [0.0, 0.1, 0.2, 0.3][action_idx // 3]
        w = [-0.5, 0.0, 0.5][action_idx % 3]
        next_state, reward, done, _ = env.step((v, w))
        agent.replay.push(state, action_idx, reward, next_state, done)
        agent.train_step(batch_size)
        state = next_state
        total_reward += reward
        if done:
            break
    agent.update_target()
    epsilon = max(epsilon * epsilon_decay, epsilon_min)
    print(f"Ep {ep}: recompensa={total_reward:.2f}")

torch.save(agent.q_net.state_dict(), 'data/Q_model.pth', _use_new_zipfile_serialization=False)

