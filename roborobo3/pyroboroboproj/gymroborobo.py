import gym
import numpy as np
import pyroborobo


class PyroboroboGym(gym.Env):

    action_space = gym.spaces.Box(np.array([-1., -1.], dtype=np.float32), np.array([1., 1.], dtype=np.float32))
    observation_space = gym.spaces.Box(np.array(8 * [0.], dtype=np.float32), np.array(8 * [1.], dtype=np.float32))

    def __init__(self):
        self.rob = pyroborobo.Pyroborobo('../config/pyfastwanderer.properties', None, None, None, None, {})
        self.rob.start()

    def step(self, action):
        self.rob.update(1)
        self.rob.getRobot(0)

    def reset(self):
        pass


if __name__ == "__main__":
    print('coucou')