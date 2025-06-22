import gymnasium as gym
from typing import Optional
import os


def make_ant_env(
    seed: int,
    idx: int,
    capture_video: bool,
    run_name: Optional[str] = None,
    prefix: str = "",
):
    def thunk():
        env = gym.make(
            "Ant-v5",
            xml_file="./simulation/robot_model/spot.xml",
            render_mode="rgb_array",
            forward_reward_weight=1,
            ctrl_cost_weight=0.05,
            contact_cost_weight=5e-4,
            healthy_reward=1,
            main_body=1,
            healthy_z_range=(0.195, 0.75),
            include_cfrc_ext_in_observation=True,
            exclude_current_positions_from_observation=False,
            reset_noise_scale=0.1,
            frame_skip=25,
            max_episode_steps=1000,
        )
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
