import gymnasium as gym
from ai.env.env import register_common_environment_parameters
from ai.env.spot_env import SpotEnv


def import_envs_collection() -> None:
    register_common_environment_parameters()

    # register environments so they can be built using gym.make
    gym.register(
        id="Spot-v0",
        entry_point=SpotEnv,
        max_episode_steps=500,
    )
