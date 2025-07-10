import gymnasium as gym
import pytest
import sys
from ai.env.env import build_vectorized_envs
from ai.env.env_registrations import import_envs_collection
from ai.parameters.registry import ParameterRegistry
from typing import Callable, Tuple


@pytest.fixture
def setup(autouse=True):
    import_envs_collection()


# purpose of those tests is to check if the environment creation does not yield errors
@pytest.mark.parametrize(
    "env_spec", [("Spot-v0", "./simulation/robot_model/spot_mini.xml")]
)
def test_build_model(env_spec: Tuple[str, str], setup: Callable) -> None:
    ParameterRegistry.set_parameter_value("environment", "xml_file", env_spec[1])
    ParameterRegistry.set_parameter_value("environment", "env_name", env_spec[0])
    envs = build_vectorized_envs()
    assert envs is not None
    assert envs.num_envs == 1
    envs.close()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__]))
