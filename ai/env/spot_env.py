import gymnasium as gym
from dataclasses import dataclass


import numpy as np
from gymnasium import utils
from gymnasium.envs.mujoco import MujocoEnv
from gymnasium.spaces import Box
from ai.parameters.registry import register_parameters


@register_parameters("environment")
class SpotEnvParameters:
    vel_x: np.float64 = 0.0
    vel_y: np.float64 = 0.0
    yaw_rate: np.float64 = 0.0
    target_height: np.float64 = 0.0


# Courtesy of https://github.com/denisgriaznov/CustomMuJoCoEnviromentForRL
class SpotEnv(MujocoEnv, utils.EzPickle):
    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ]
    }

    def __init__(
        self, xml_file: str, parameters: SpotEnvParameters, episode_len=500, **kwargs
    ):
        utils.EzPickle.__init__(self, **kwargs)
        self.parameters = parameters
        MujocoEnv.__init__(self, xml_file, 5, observation_space=None, **kwargs)
        self.previous_action = np.zeros((self.model.nu,), dtype=np.float64)
        self.observation_space = Box(
            low=-np.inf, high=np.inf, shape=(48,), dtype=np.float64
        )

        self.step_number = 0
        self.episode_len = episode_len
        self.frame_skip = 5

    def step(self, a):
        reward = 0.785398
        self.do_simulation(a, self.frame_skip)
        self.step_number += 1
        self.previous_action = a
        obs = self._get_obs()
        done = bool(not np.isfinite(obs).all() or (obs[2] < 0))
        truncated = self.step_number > self.episode_len

        if self.render_mode == "human":
            self.render()
        return obs, reward, done, truncated, {}

    def reset_model(self):
        self.step_number = 0

        qpos = self.init_qpos + self.np_random.uniform(
            size=self.model.nq, low=-0.01, high=0.01
        )
        qvel = self.init_qvel + self.np_random.uniform(
            size=self.model.nv, low=-0.01, high=0.01
        )
        self.set_state(qpos, qvel)
        return self._get_obs()

    def _get_obs(self):
        w, x, y, z = self.data.sensor("Body_Quat").data
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = np.arcsin(2 * (w * y - z * x))

        joint_pos = np.concatenate(
            (
                self.data.sensor("fl.hx_pos").data,
                self.data.sensor("fl.hy_pos").data,
                self.data.sensor("fl.kn_pos").data,
                self.data.sensor("fr.hx_pos").data,
                self.data.sensor("fr.hy_pos").data,
                self.data.sensor("fr.kn_pos").data,
                self.data.sensor("hl.hx_pos").data,
                self.data.sensor("hl.hy_pos").data,
                self.data.sensor("hl.kn_pos").data,
                self.data.sensor("hr.hx_pos").data,
                self.data.sensor("hr.hy_pos").data,
                self.data.sensor("hr.kn_pos").data,
            ),
            axis=0,
        )
        joint_vel = np.concatenate(
            (
                self.data.sensor("fl.hx_vel").data,
                self.data.sensor("fl.hy_vel").data,
                self.data.sensor("fl.kn_vel").data,
                self.data.sensor("fr.hx_vel").data,
                self.data.sensor("fr.hy_vel").data,
                self.data.sensor("fr.kn_vel").data,
                self.data.sensor("hl.hx_vel").data,
                self.data.sensor("hl.hy_vel").data,
                self.data.sensor("hl.kn_vel").data,
                self.data.sensor("hr.hx_vel").data,
                self.data.sensor("hr.hy_vel").data,
                self.data.sensor("hr.kn_vel").data,
            ),
            axis=0,
        )
        obs = np.concatenate(
            (
                self.data.sensor("Body_Vel").data,
                self.data.sensor("Body_Gyro").data,
                np.array([roll]),
                np.array([pitch]),
                joint_pos,
                joint_vel,
                self.previous_action,
                np.array(
                    [
                        self.parameters.vel_x,
                        self.parameters.vel_y,
                        self.parameters.yaw_rate,
                    ]
                ),  # Placeholder target vx,vy, yaw_rate
                np.array([self.parameters.target_height]),  # placeholder target z
            ),
            axis=0,
        )
        return obs


# Register the custom environment
gym.register(
    id="Spot-v0",
    entry_point=SpotEnv,
    kwargs={"episode_len": 500},
)
