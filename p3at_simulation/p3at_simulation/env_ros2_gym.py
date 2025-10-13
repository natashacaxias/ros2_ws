import numpy as np

class Ros2Env:
    def __init__(self):
        self.goal = np.array([2.0, 2.0])
        self.pose = np.zeros(3)  # x, y, theta
        self.done = False

    def reset(self):
        self.pose = np.zeros(3)
        self.done = False
        return self._get_state()

    def _get_state(self):
        erro = self.goal - self.pose[:2]
        dist = np.linalg.norm(erro)
        return np.concatenate([self.pose, erro, [dist]])

    def step(self, action):
        # aplica a ação (v, rot)
        v, w = action
        dt = 0.1
        self.pose[0] += v * np.cos(self.pose[2]) * dt
        self.pose[1] += v * np.sin(self.pose[2]) * dt
        self.pose[2] += w * dt

        erro = self.goal - self.pose[:2]
        dist = np.linalg.norm(erro)
        reward = -dist
        if dist < 0.1:
            reward += 100
            self.done = True

        next_state = self._get_state()
        return next_state, reward, self.done, {}
