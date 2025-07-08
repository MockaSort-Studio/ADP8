import gymnasium as gym
import os
from ai.parameters.registry import ParameterRegistry
from typing import Any, Callable, Dict, Optional, Type

_ENV_COMMON_PARAMETERS: Dict[str, Any] = {
    "episode_len": 500,
    "xml_file": "./simulation/robot_model/spot_mini.xml",
    "capture_video": False,
}


# REFERENCE CODE
# return shapes of the observation and action spaces
def make_env(
    env_id: str,
    seed: int,
    idx: int,
    capture_video: bool,
    env_specs=None,
    run_name: Optional[str] = None,
    prefix: str = "",
):
    def thunk():

        kwargs_env_specs = {}
        if isinstance(env_specs, dict):
            kwargs_env_specs.update(env_specs)

        env = gym.make(env_id, **kwargs_env_specs)
        env = gym.wrappers.RecordEpisodeStatistics(env)
        if capture_video and idx == 0 and run_name is not None:
            env = gym.wrappers.RecordVideo(
                env,
                os.path.join(run_name, prefix + "_videos" if prefix else "videos"),
                disable_logger=True,
            )
        env.action_space.seed(seed)
        env.observation_space.seed(seed)
        return env

    return thunk


# NEW CODE


def environment_parameters(**parameters: Dict[str, Any]) -> Callable:
    def decorator(cls: Type) -> Type:
        # Register the environment and its parameters in the ParameterServer
        registered_parameters = _ENV_COMMON_PARAMETERS.copy()
        registered_parameters.update(parameters)
        ParameterRegistry.register("environment", registered_parameters)

        # Modify the class to inject the parameters
        original_init = cls.__init__

        def new_init(self, **kwargs):
            # Inject the parameter dataclass instance
            self.parameters = ParameterRegistry.get_parameters("environment")
            # Call the original __init__ method
            original_init(self, **kwargs)

        cls.__init__ = new_init
        return cls

    return decorator
