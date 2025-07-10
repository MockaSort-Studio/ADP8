import numpy as np
import gymnasium as gym
from ai.parameters.registry import ParameterRegistry

from ai.env.env import (
    build_vectorized_envs,
)
from ai.env.env_registrations import import_envs_collection

# Register common environment parameters
import_envs_collection()

ParameterRegistry.set_parameter_value(
    "environment", "xml_file", "./simulation/robot_model/spot_mini.xml"
)
ParameterRegistry.set_parameter_value("environment", "env_name", "Spot")
env = build_vectorized_envs()
# Example of running the environment for a few steps
obs, info = env.reset()
env.render()

while True:
    # action = np.random.uniform(
    #     low=-10.0, high=10.0, size=12
    # )  # Ensure proper range for random actions
    action = np.random.uniform(
        low=-10.0, high=10.0, size=(env.num_envs, 12)
    )  # Generate actions for all environments
    obs, reward, terminated, truncated, info = env.step(action)
    print(action)
    print(obs)
    # if terminated or truncated:
    # obs, info = env.reset()

env.close()
print("Environment tested successfully!")
