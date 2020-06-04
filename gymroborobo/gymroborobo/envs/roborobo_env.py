import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
from gymroborobo.pyroborobo import Roborobo


class RoboroboEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    action_space = spaces.Box([-1, -1, 0], [1, 1, 10])
    observation_space = spaces.Box([-1] * 81, [1] * 81)

    def __init__(self):
        # start roborobo
        self.roborobo = Roborobo()
        self.roborobo.start()

    def step(self, action):
        info_n = []
        obs_n, reward_n, done_n = self.roborobo.send_actions(action)
        return obs_n, reward_n, done_n, info_n

    def reset(self):
        roborobo.reset()

    def render(self, mode='human'):
        ...

    def close(self):
        roborobo.close()
