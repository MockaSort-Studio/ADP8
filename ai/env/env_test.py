import numpy as np
from ai.core.parameters import ParameterRegistry

from ai.env.env import build_vectorized_envs, build_env
from ai.env.env_registrations import import_envs_collection

# Register common environment parameters
import_envs_collection()

ParameterRegistry.set_parameter_value(
    "environment", "xml_file", "./simulation/robot_model/spot_mini.xml"
)
ParameterRegistry.set_parameter_value("environment", "render_mode", "human")
ParameterRegistry.set_parameter_value("environment", "env_name", "Spot")
ParameterRegistry.set_parameter_value("environment", "num_envs", 1)
env = build_env()
# env = build_vectorized_envs()

# Example of running the environment for a few steps
obs, info = env.reset()
env.render()

while True:
    # action = np.random.uniform(
    #     low=-10.0, high=10.0, size=12
    # )  # Ensure proper range for random actions
    # action = np.random.uniform(
    #     low=-0.0, high=0.0, size=(env.num_envs, 12)
    # )  # Generate actions for all environments
    action = np.array(
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float32
    )  # Example action
    obs, reward, terminated, truncated, info = env.step(action)
    print(action)
    print(obs)
# if terminated or truncated:
# obs, info = env.reset()

env.close()
print("Environment tested successfully!")
