import gymnasium as gym


import numpy as np
from gymnasium import utils
from gymnasium.envs.mujoco import MujocoEnv
from gymnasium.spaces import Box

import os


# Courtesy of https://github.com/denisgriaznov/CustomMuJoCoEnviromentForRL
class SpotEnv(MujocoEnv, utils.EzPickle):
    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ]
    }

    def __init__(self, episode_len=500, **kwargs):
        utils.EzPickle.__init__(self, **kwargs)
        self.previous_action = np.zeros((12,), dtype=np.float64)
        MujocoEnv.__init__(
            self,
            "./simulation/robot_model/spot_mini.xml",
            5,
            observation_space=None,
            **kwargs
        )
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
                np.array([0.0, 0.0, 0.0]),  # Placeholder target vx,vy, yaw_rate
                np.array([0.0]),  # placeholder target z
            ),
            axis=0,
        )
        return obs


# Register the custom environment
gym.register(
    id="Spot-v0",  # Unique ID for the environment
    entry_point="env.spot_env:SpotEnv",  # Path to the environment class
    kwargs={"episode_len": 500},  # Optional arguments passed to the environment
)
