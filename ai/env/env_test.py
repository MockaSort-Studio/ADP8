import numpy as np

from ai.parameters.registry import ParameterRegistry
from ai.env.env import (
    register_common_environment_parameters,
    build_env,
)

# Register common environment parameters
register_common_environment_parameters()

ParameterRegistry.set_parameter_value(
    "environment", "xml_file", "./simulation/robot_model/spot_mini.xml"
)
ParameterRegistry.set_parameter_value(
    "environment", "env_path", "ai.env.spot_env.SpotEnv"
)
env = build_env()
# Example of running the environment for a few steps
obs, info = env.reset()
env.render()

while True:
    action = np.random.uniform(
        low=-10.0, high=10.0, size=12
    )  # Ensure proper range for random actions
    obs, reward, terminated, truncated, info = env.step(action)
    print(action)
    print(obs)
    # if terminated or truncated:
    # obs, info = env.reset()

env.close()
print("Environment tested successfully!")
