import gym
import numpy as np
from tensorforce.agents import Agent
from tensorforce.environments import OpenAIGym

import tqdm
from ppoyield.ppoyield import ppo, MustReset
import spinup.algos.pytorch.ppo.core as core
from spinup.utils.mpi_tools import mpi_fork


class BigEnv(gym.Env):
    action_space = gym.spaces.Box(np.array([0]), np.array([1]))
    observation_space = gym.spaces.Box(np.array([0]), np.array([1]))

    def __init__(self, n_agents, maxit=300):
        self.n_agents = n_agents
        self.agents_obs = np.zeros((self.n_agents, self.observation_space.shape[0]))
        self.agents_actions = np.zeros((self.n_agents, self.observation_space.shape[0]))
        self.maxit = maxit
        self.iter = 0

    def reset(self):
        self.agents_obs = np.zeros((self.n_agents, self.observation_space.shape[0]))
        self.agents_actions = np.zeros((self.n_agents, self.observation_space.shape[0]))
        self.iter = 0
        return np.zeros((2,))

    def get_agent_obs(self, i: int):
        return self.agents_obs[i]

    def set_agent_actions(self, i: int, actions: np.array):
        self.agents_actions[i] = actions

    def step(self, actions):
        self.iter += 1
        actions = actions.flatten()
        rewards = actions
        info = {}
        obs = np.array([[1] for _ in range(self.n_agents)])
        done = [self.iter >= self.maxit] * self.n_agents
        return obs, rewards, done, info



def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--env', type=str, default='HalfCheetah-v2')
    parser.add_argument('--hid', type=int, default=64)
    parser.add_argument('--l', type=int, default=2)
    parser.add_argument('--gamma', type=float, default=0.99)
    parser.add_argument('--seed', '-s', type=int, default=0)
    parser.add_argument('--cpu', type=int, default=1)
    parser.add_argument('--steps', type=int, default=4000)
    parser.add_argument('--epochs', type=int, default=50)
    parser.add_argument('--exp_name', type=str, default='ppo')
    args = parser.parse_args()

    from spinup.utils.run_utils import setup_logger_kwargs

    logger_kwargs = setup_logger_kwargs(args.exp_name, args.seed)

    mpi_fork(args.cpu)

    maxit = 100000
    nb_agents = 10
    environment = BigEnv(nb_agents, maxit)

    agents = [
        ppo(lambda: environment) for _ in range(nb_agents)
    ]
    print('agents built')

    # Init agents
    for agent in agents:
        status = next(agent)
        print(status)
        assert(isinstance(status, MustReset))
    statess = environment.reset()
    actions = []
    for i, agent in enumerate(agents):
        a = agent.send(environment.get_agent_obs(i))
        actions.append(a)

    for curit in tqdm.tqdm(range(maxit)):
        terminal = False
        must_reset = False
        # Episode timestep
        statess, rewards, terminals, info = environment.step(np.array(actions))
        actions = []
        for agent, states, terminal, reward in zip(agents, statess, terminals, rewards):
            action = agent.send((states, reward, terminal, {}))
            if isinstance(action, MustReset):
                must_reset = True
            actions.append(action)
        if must_reset:
            statess = environment.reset()
            actions = []
            for i, agent in enumerate(agents):
                a = agent.send(environment.get_agent_obs(i))
                actions.append(a)
        if terminal:
            break
        if curit % 10000 < 10:
            print(rewards)


if __name__ == "__main__":
    main()



