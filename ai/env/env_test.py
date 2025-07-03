import numpy as np

import gymnasium as gym

env = gym.make(
    "Ant-v5",
    # xml_file="./simulation/robot_model/spot_mini.xml",
    render_mode="human",
    forward_reward_weight=0,
    ctrl_cost_weight=0,
    contact_cost_weight=0,
    healthy_reward=0,
    main_body=1,
    healthy_z_range=(0, np.inf),
    include_cfrc_ext_in_observation=True,
    exclude_current_positions_from_observation=False,
    reset_noise_scale=0,
    frame_skip=1,
    max_episode_steps=1000,
)

# env = gym.wrappers.RecordVideo(
#     env,
#     os.path.join("dio_canguro", "madonna_videos"),
#     disable_logger=True,
# )
# Example of running the environment for a few steps
obs, info = env.reset()
env.render()

while True:
    action = env.action_space.sample()  # Replace with your agent's action
    obs, reward, terminated, truncated, info = env.step(action)
    print(action)
    print(obs)
    if terminated or truncated:
        obs, info = env.reset()

env.close()
print("Environment tested successfully!")

# from urdf2mjcf import run

# run(
#     urdf_path="./simulation/robot_model/spot.xml",
#     mjcf_path="/home/vscode/spot.mjcf",
#     copy_meshes=True,
# )
