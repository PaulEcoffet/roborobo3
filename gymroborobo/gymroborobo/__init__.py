from gym.envs.registration import register

register(
    id='roborobo-v0',
    entry_point='gym_foo.envs:RoboroboEnv',
)
