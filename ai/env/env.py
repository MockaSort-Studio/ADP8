import gymnasium as gym
from typing import Optional
import os


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
