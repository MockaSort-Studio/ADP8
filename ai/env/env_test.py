import numpy as np

import gymnasium as gym


import numpy as np

from ai.utils.module_loader import import_module
from ai.parameters.registry import ParameterRegistry

# to be handled differently
import_module(
    "ai.env.spot_env",
)

env = gym.make(
    "Spot-v0",
    render_mode="human",
    parameters=ParameterRegistry.get_parameters("environment"),
    xml_file="./simulation/robot_model/spot_mini.xml",
)

# Example of running the environment for a few steps
obs, info = env.reset()
env.render()

# while True:
#     action = np.random.uniform(
#         low=-60.0, high=60.0, size=12
#     )  # Ensure proper range for random actions
#     obs, reward, terminated, truncated, info = env.step(action)
#     print(action)
#     print(obs)
#     # if terminated or truncated:
#     obs, info = env.reset()

env.close()
print("Environment tested successfully!")
