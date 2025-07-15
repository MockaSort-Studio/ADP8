import gymnasium as gym
import os
import numpy as np
from gymnasium import utils
from gymnasium.envs.mujoco import MujocoEnv
from ai.core.parameters import ParameterRegistry
from typing import Any, Dict, Optional, Tuple


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
_COMMON_PARAMETERS: Dict[str, Any] = {
    "env_name": "",
    "episode_len": 500,
    "xml_file": "",
    "capture_video": False,
    "frame_skip": 5,
    "render_mode": "rgb_array",
    "seed": 42,
    "num_envs": 1,
}


def register_common_environment_parameters() -> None:
    """
    Register default parameters for the environment.
    This function can be called to ensure that the default parameters are set.
    """
    ParameterRegistry.register("environment", _COMMON_PARAMETERS)


def build_env() -> gym.Env:
    """
    Build a single environment
    """
    env_name = ParameterRegistry.get_parameter_value("environment", "env_name")
    capture_video = ParameterRegistry.get_parameter_value(
        "environment", "capture_video"
    )
    seed = ParameterRegistry.get_parameter_value("environment", "seed")
    run_name = env_name + f"_{seed}_{int(os.getpid())}" if env_name else None
    env = gym.make(id=env_name, render_mode="rgb_array")
    env = gym.wrappers.RecordEpisodeStatistics(env)
    if capture_video:
        env = gym.wrappers.RecordVideo(
            env,
            os.path.join(run_name, "videos"),
            disable_logger=True,
        )
    env.action_space.seed(seed)
    env.observation_space.seed(seed)
    return env


# putting this aside until first impl of loss vegas is done
def build_vectorized_envs() -> gym.vector.SyncVectorEnv:
    envs = gym.vector.SyncVectorEnv(
        [
            build_env
            for _ in range(
                ParameterRegistry.get_parameter_value("environment", "num_envs")
            )
        ]
    )

    return envs


# ## BaseGymnasiumEnv

# BaseGymnasiumEnv is an abstract base class for creating custom Gymnasium environments using the Mujoco physics engine.
# It provides a structured framework for defining environment behavior, including initialization, stepping through the environment,
# resetting, and rendering. Subclasses are expected to implement specific methods to customize the environment's behavior.

# ## Metadata
# - `render_modes`: Supported rendering modes include:
#     - `human`: Render the environment for human visualization.
#     - `rgb_array`: Render the environment as an RGB image array.
#     - `depth_array`: Render the environment as a depth image array.

# ## Attributes
# - `parameters`: Configuration parameters for the environment, retrieved from `ParameterRegistry`.
# - `action_space`: The action space of the environment, defined as a discrete space based on the Mujoco model's degrees of freedom.
# - `observation_space`: The observation space of the environment, initialized by the subclass.
# - `episode_len`: The maximum length of an episode.
# - `step_number`: The current step number in the episode.

# ## Methods

# ### `__init__(**kwargs: Dict[str, Any]) -> None`
# Initializes the environment with parameters from `ParameterRegistry` and sets up the Mujoco model, action space, and observation space.

# ### `step(a: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, dict]`
# Performs a single step in the environment.
# - **Parameters**:
#     - `a`: The action to be taken.
# - **Returns**:
#     - `obs`: The observation after the step.
#     - `reward`: The reward obtained from the step.
#     - `done`: Whether the episode has ended.
#     - `truncated`: Whether the episode was truncated.
#     - `info`: Additional information.

# ### `reset_model() -> np.ndarray`
# Resets the environment to its initial state.
# - **Returns**:
#     - The initial observation.

# ### `_get_obs() -> np.ndarray`
# Retrieves the current observation. This method calls `get_obs_impl()`.

# ### `init_observation_space() -> gym.spaces.Space`
# Abstract method to initialize the observation space. Must be implemented by subclasses.

# ### `get_obs_impl() -> np.ndarray`
# Abstract method to define custom observation logic. Must be implemented by subclasses.

# ### `is_done(obs: np.ndarray) -> bool`
# Abstract method to define episode termination conditions. Must be implemented by subclasses.

# ### `is_truncated() -> bool`
# Abstract method to define episode truncation conditions. Must be implemented by subclasses.

# ### `reward(obs: np.ndarray, action: np.ndarray) -> float`
# Defines the reward logic. Can be overridden by subclasses.
# - **Returns**:
#     - A float value representing the reward (default is `0.0`).

# ### `step_pre(action: np.ndarray) -> None`
# Optional method to define custom logic before the simulation step. Can be overridden by subclasses.

# ### `step_post(action: np.ndarray) -> None`
# Optional method to define custom logic after the simulation step. Can be overridden by subclasses.

# ---

# *Documented with care by GitHub Copilot.*


# Mujoco env tutorial: https://github.com/denisgriaznov/CustomMuJoCoEnviromentForRL
class BaseGymnasiumEnv(MujocoEnv, utils.EzPickle):
    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ]
    }

    # if you need to run further operation during initialization, just implement a new one and explicitly invoke parent class __init__
    def __init__(self, **kwargs: Dict[str, Any]) -> None:
        self.parameters = ParameterRegistry.get_parameters("environment")
        utils.EzPickle.__init__(self, **kwargs)
        MujocoEnv.__init__(
            self,
            model_path=self.parameters.xml_file,
            frame_skip=self.parameters.frame_skip,
            observation_space=None,
            render_mode=self.parameters.render_mode,
        )

        self.action_space = self.init_action_space()
        self.observation_space = self.init_observation_space()
        self.observation_space.seed(self.parameters.seed)
        self.action_space.seed(self.parameters.seed)
        self.episode_len = self.parameters.episode_len
        self.step_number = 0

    def step(self, a: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, dict]:
        self.step_pre(action=a)
        self.do_simulation(a, self.frame_skip)
        self.step_number += 1
        self.step_post(action=a)
        obs = self._get_obs()
        reward = self.reward(obs=obs, action=a)
        done = self.is_done(obs)
        truncated = self.is_truncated()

        if self.render_mode == "human":
            self.render()
        return obs, reward, done, truncated, {}

    def reset_model(self) -> np.ndarray:
        self.step_number = 0

        qpos = self.init_qpos + self.np_random.uniform(
            size=self.model.nq, low=-0.01, high=0.01
        )
        qvel = self.init_qvel + self.np_random.uniform(
            size=self.model.nv, low=-0.01, high=0.01
        )
        self.set_state(qpos, qvel)
        return self._get_obs()

    def _get_obs(self) -> np.ndarray:
        return self.get_obs_impl()

    def init_action_space(self) -> gym.spaces.Space:
        return gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.model.nu,),
            dtype=np.float32,
        )

    def init_observation_space(self) -> gym.spaces.Space:
        raise NotImplementedError(
            f"{self.init_observation_space.__name__}  method should be implemented in subclasses."
        )

    def get_obs_impl(self) -> np.ndarray:
        """
        Override this method to implement custom observation logic.
        """
        raise NotImplementedError(
            f"{self.get_obs_impl.__name__} method should be implemented in subclasses."
        )

    def is_done(
        self,
        obs: np.ndarray,
    ) -> bool:
        """
        Override this method to implement episodes truncation conditions.
        """
        raise NotImplementedError(
            f"{self.is_done.__name__} method should be implemented in subclasses."
        )

    def is_truncated(self) -> bool:
        """
        Override this method to implement episodes truncation conditions.
        """
        raise NotImplementedError(
            f"{self.is_done.__name__} method should be implemented in subclasses."
        )

    def reward(self, obs: np.ndarray, action: np.ndarray) -> float:
        """
        Override this method to implement custom reward logic.
        """
        return 0.0

    def step_pre(self, action: np.ndarray) -> None:
        """
        Override this method to implement custom step logic before the simulation step.
        """
        pass

    def step_post(self, action: np.ndarray) -> None:
        """
        Override this method to implement custom step logic after the simulation step.
        """
        pass
