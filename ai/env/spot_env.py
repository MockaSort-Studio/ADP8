import gymnasium as gym
import numpy as np
from typing import Dict, override

from ai.env.env import BaseGymnasiumEnv
from ai.core.parameters import declare_parameters


class Reward:
    def __init__(
        self,
        target_height: float = 0.5,
        target_vel_x: float = 0.0,
        target_vel_y: float = 0.0,
        target_yaw_rate: float = 0.0,
    ):
        self.target_height = target_height
        self.target_vel_x = target_vel_x
        self.target_vel_y = target_vel_y
        self.target_yaw_rate = target_yaw_rate

    def _general_reward_fn(self, target: np.ndarray, current: np.ndarray) -> float:
        """
        Computes a general reward based on the difference between target and current values.
        """
        diff = np.sum(np.square(target - current))
        print(f"Target: {target}, Current: {current}, Difference: {diff}")
        reward = np.exp(-diff)
        return reward

    def _action_rate_penalty(self, previous: np.ndarray, current: np.ndarray) -> float:
        return float(np.sum(np.square(current - previous)))

    def _pose_similarity_reward(
        self, default_qpos: np.ndarray, qpos: np.ndarray
    ) -> float:
        return float(np.sum(np.abs(qpos - default_qpos)))

    # TODO: figure out how to get current robot height and apply height penalty
    def compute(self, inputs: Dict[str, np.ndarray]) -> float:
        linear_vel_reward = self._general_reward_fn(
            inputs["commands"][:2], inputs["tracked_linear"][:2]
        )
        print(f"Linear velocity reward: {linear_vel_reward}")
        angular_vel_reward = self._general_reward_fn(
            inputs["commands"][2], inputs["tracked_yaw_rate"]
        )
        print(f"Angular velocity reward: {angular_vel_reward}")
        action_penalty = self._action_rate_penalty(
            inputs["previous_action"], inputs["action"]
        )
        print(f"Action penalty: {action_penalty}")
        vel_z_penalty = float(np.square(inputs["tracked_linear"][2]))
        print(f"Vertical velocity penalty: {vel_z_penalty}")
        roll_and_pitch_stability_penalty = float(
            np.square(inputs["roll"] + inputs["pitch"])
        )
        print(f"Roll and pitch stability penalty: {roll_and_pitch_stability_penalty}")
        pose_similarity_reward = self._pose_similarity_reward(
            inputs["default_qpos"], inputs["qpos"]
        )
        reward = (
            linear_vel_reward
            + angular_vel_reward
            + pose_similarity_reward
            - action_penalty
            - vel_z_penalty
            - roll_and_pitch_stability_penalty
        )
        print(f"Total reward: {reward}")
        return reward


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
        self._reward = Reward(
            target_height=self.parameters.target_height,
            target_vel_x=self.parameters.vel_x,
            target_vel_y=self.parameters.vel_y,
            target_yaw_rate=self.parameters.yaw_rate,
        )
        self.previous_action = np.zeros(12, dtype=np.float32)
        self.default_qpos = self._get_joint_configuration()

    def _get_joint_configuration(self) -> np.ndarray:
        qpos = np.concatenate(
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
        return qpos

    def _unpack_observations(self, obs: np.ndarray) -> Dict[str, np.ndarray]:
        commands = obs[-4:]
        tracked_linear = obs[:3]
        previous_action = obs[-16:-4]
        tracked_yaw_rate = np.array([obs[5]])
        roll = np.array([obs[6]])
        pitch = np.array([obs[7]])
        qpos = obs[8:20]

        return {
            "commands": commands,
            "tracked_linear": tracked_linear,
            "previous_action": previous_action,
            "tracked_yaw_rate": tracked_yaw_rate,
            "roll": roll,
            "pitch": pitch,
            "qpos": qpos,
            "default_qpos": self.default_qpos,
        }

    @override
    def init_observation_space(self) -> gym.spaces.Space:
        return gym.spaces.Box(low=-np.inf, high=np.inf, shape=(48,), dtype=np.float32)

    @override
    def step_pre(self, action: np.ndarray) -> None:
        action = self.default_qpos + action

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
    def reward(self, obs: np.ndarray, action: np.ndarray) -> float:
        unpacked_obs = self._unpack_observations(obs)
        unpacked_obs["action"] = action
        return self._reward.compute(inputs=unpacked_obs)

    @override
    def get_obs_impl(self):
        w, x, y, z = self.data.sensor("Body_Quat").data
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = np.arcsin(2 * (w * y - z * x))

        joint_pos = self._get_joint_configuration()
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
