import gymnasium as gym
import numpy as np
from typing import override

from ai.env.env import BaseGymnasiumEnv
from ai.core.parameters import declare_parameters


@declare_parameters(
    parameter_set_name="environment",
    vel_x=0.0,
    vel_y=0.0,
    yaw_rate=0.0,
    target_height=0.5,
)
class SpotEnv(BaseGymnasiumEnv):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.previous_action = np.zeros(12, dtype=np.float32)

    @override
    def init_observation_space(self) -> gym.spaces.Space:
        return gym.spaces.Box(low=-np.inf, high=np.inf, shape=(48,), dtype=np.float32)

    @override
    def is_done(self, obs: np.ndarray) -> bool:
        return bool(not np.isfinite(obs).all())

    @override
    def is_truncated(self) -> bool:
        return self.step_number > self.episode_len

    @override
    def step_post(self, action: np.ndarray) -> None:
        self.previous_action = action

    @override
    def get_obs_impl(self):
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
                ),
                np.array([self.parameters.target_height]),
            ),
            axis=0,
        )
        return obs
