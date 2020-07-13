import gym
import numpy as np
from tensorforce.agents import Agent
from tensorforce.environments import OpenAIGym

import tqdm


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
        obs = np.array([[1],
                        [1]])
        done = self.iter >= self.maxit
        return obs, rewards, done, info


def create_ppo(environment):
    return Agent.create(
            agent='ppo',
            # environment data
            environment=environment,
        network='auto',
        # Optimization
        batch_size=10, update_frequency=2, learning_rate=1e-3, subsampling_fraction=0.2,
        optimization_steps=5,
        # Reward estimation
        likelihood_ratio_clipping=0.2, discount=0.99, estimate_terminal=False,
        # Critic
        critic_network='auto',
        critic_optimizer=dict(optimizer='adam', multi_step=10, learning_rate=1e-3),
        # Preprocessing
        preprocessing=None,
        # Exploration
        exploration=0.0, variable_noise=0.0,
        # Regularization
        l2_regularization=0.0, entropy_regularization=0.0,
        # TensorFlow etc
        name='agent', device=None, parallel_interactions=1, seed=None, execution=None, saver=None,
        summarizer=None, recorder=None
    )


def main():
    maxit = 1000000
    environment = OpenAIGym(BigEnv(2, maxit), max_episode_steps=maxit)

    agents = [
        create_ppo(environment),
        create_ppo(environment)
        ]
    print('agents built')
    statess = environment.reset()
    for curit in tqdm.tqdm(range(maxit)):
        terminal = False
        # Episode timestep
        actions = []
        for agent, states in zip(agents, statess):
            states = np.reshape(states, (1,))
            action = agent.act(states=np.asarray(states))
            actions.append(action)
        if curit % 10000 < 10:
            print('states', statess)
            print('action', actions)
        statess, terminals, rewards = environment.execute(actions=np.array(actions))
        terminals = [terminals] * len(agents)
        if curit % 10000 < 10:
            print('reward', rewards)
        for agent, states, terminal, reward in zip(agents, statess, terminals, rewards):
            agent.observe(terminal=terminal, reward=reward)
        if terminal:
            break

    for agent in agents:
        agent.close()
    environment.close()


if __name__ == "__main__":
    main()



