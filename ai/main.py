import argparse
import os
import time
from datetime import datetime

import gymnasium as gym
import torch
import torchmetrics
from ai.models.agent import PPOLightningAgent
from ai.loss.ppo_loss import PPOLoss
from ai.utils.utils import linear_annealing, parse_args, test
from ai.train.train import train
from ai.env.cartpole_env import make_cartpole_env
from lightning.fabric import Fabric
from lightning.fabric.loggers import TensorBoardLogger
from typing import Tuple
from dataclasses import dataclass


@dataclass
class TrainingCache:
    observations: torch.Tensor
    actions: torch.Tensor
    logprobs: torch.Tensor
    rewards: torch.Tensor
    terminateds: torch.Tensor
    values: torch.Tensor


def setup_fabric(args: argparse.Namespace, logger: TensorBoardLogger) -> Fabric:
    # Initialize Fabric
    fabric = Fabric(
        accelerator=args.accelerator,
        devices=args.devices,
        strategy=args.strategy,
        loggers=logger,
    )
    fabric.launch()
    fabric.seed_everything(args.seed)
    torch.backends.cudnn.deterministic = args.torch_deterministic

    # Log hyperparameters
    fabric.logger.experiment.add_text(
        "hyperparameters",
        "|param|value|\n|-|-|\n{}".format(
            "\n".join([f"|{key}|{value}|" for key, value in vars(args).items()])
        ),
    )
    return fabric


def setup_environment(
    args: argparse.Namespace, fabric: Fabric
) -> gym.vector.SyncVectorEnv:
    rank = fabric.global_rank
    logger = fabric.logger
    # Environment setup
    envs = gym.vector.SyncVectorEnv(
        [
            make_cartpole_env(
                args.env_id,
                args.seed + rank * args.num_envs + i,
                rank,
                args.capture_video,
                logger.log_dir,
                "train",
            )
            for i in range(args.num_envs)
        ]
    )
    assert isinstance(
        envs.single_action_space, gym.spaces.Discrete
    ), "only discrete action space is supported"
    return envs


def setup_agent_and_optimizer(
    args: argparse.Namespace, envs: gym.vector.SyncVectorEnv, fabric: Fabric
) -> Tuple[PPOLightningAgent, torch.optim.Optimizer]:
    ppo_loss = PPOLoss(
        vf_coef=args.vf_coef,
        ent_coef=args.ent_coef,
        clip_coef=args.clip_coef,
        clip_vloss=args.clip_vloss,
        normalize_advantages=args.normalize_advantages,
    )
    agent = PPOLightningAgent(
        envs,
        ppo_loss,
        act_fun=args.activation_function,
        ortho_init=args.ortho_init,
    )

    optimizer = agent.configure_optimizers(args.learning_rate)
    agent, optimizer = fabric.setup(agent, optimizer)

    return agent, optimizer


def initialize_training_cache(
    args: argparse.Namespace,
    envs: gym.vector.SyncVectorEnv,
    device: torch.device,
) -> TrainingCache:
    return TrainingCache(
        observations=torch.zeros(
            (args.num_steps, args.num_envs) + envs.single_observation_space.shape,
            device=device,
        ),
        actions=torch.zeros(
            (args.num_steps, args.num_envs) + envs.single_action_space.shape,
            device=device,
        ),
        logprobs=torch.zeros((args.num_steps, args.num_envs), device=device),
        rewards=torch.zeros((args.num_steps, args.num_envs), device=device),
        terminateds=torch.zeros((args.num_steps, args.num_envs), device=device),
        values=torch.zeros((args.num_steps, args.num_envs), device=device),
    )


def training_loop(
    args: argparse.Namespace,
    local_training_cache: TrainingCache,
    envs: gym.vector.SyncVectorEnv,
    agent: PPOLightningAgent,
    optimizer: torch.optim.Optimizer,
    fabric: Fabric,
) -> None:

    # Global variables
    device = fabric.device
    world_size = fabric.world_size
    global_step = 0
    start_time = time.time()
    num_episodes = args.total_timesteps // int(
        args.num_envs * args.num_steps * world_size
    )

    # Player metrics
    rew_avg = torchmetrics.MeanMetric().to(device)
    ep_len_avg = torchmetrics.MeanMetric().to(device)

    # Get the first environment observation and start the optimization
    next_obs = torch.tensor(envs.reset(seed=args.seed)[0], device=device)
    next_terminated = torch.zeros(args.num_envs, device=device)
    for episode in range(1, num_episodes + 1):
        # Learning rate annealing
        if args.anneal_lr:
            linear_annealing(optimizer, episode, num_episodes, args.learning_rate)
        fabric.log("Info/learning_rate", optimizer.param_groups[0]["lr"], global_step)

        for step in range(0, args.num_steps):
            global_step += args.num_envs * world_size
            local_training_cache.observations[step] = next_obs
            local_training_cache.terminateds[step] = next_terminated

            # Sample an action given the observation received by the environment
            with torch.no_grad():
                action, logprob, _, value = agent.model.get_action_and_value(next_obs)
                local_training_cache.values[step] = value.flatten()
            local_training_cache.actions[step] = action
            local_training_cache.logprobs[step] = logprob

            # Single environment step
            next_obs, reward, terminated, truncated, info = envs.step(
                action.cpu().numpy()
            )
            terminated = torch.logical_or(
                torch.tensor(terminated), torch.tensor(truncated)
            )
            local_training_cache.rewards[step] = torch.tensor(
                reward, device=device, dtype=torch.float32
            ).view(-1)
            next_obs, next_terminated = torch.tensor(
                next_obs, device=device
            ), terminated.to(device)

            if "final_info" in info:
                for i, agent_final_info in enumerate(info["final_info"]):
                    if agent_final_info is not None and "episode" in agent_final_info:
                        fabric.print(
                            f"Rank-0: global_step={global_step}, reward_env_{i}={agent_final_info['episode']['r'][0]}"
                        )
                        rew_avg(agent_final_info["episode"]["r"][0])
                        ep_len_avg(agent_final_info["episode"]["l"][0])

        # Sync the metrics
        rew_avg_reduced = rew_avg.compute()
        if not rew_avg_reduced.isnan():
            fabric.log("Rewards/rew_avg", rew_avg_reduced, global_step)
        ep_len_avg_reduced = ep_len_avg.compute()
        if not ep_len_avg_reduced.isnan():
            fabric.log("Game/ep_len_avg", ep_len_avg_reduced, global_step)
        rew_avg.reset()
        ep_len_avg.reset()

        # Estimate returns with GAE (https://arxiv.org/abs/1506.02438)
        returns, advantages = agent.model.estimate_returns_and_advantages(
            local_training_cache.rewards,
            local_training_cache.values,
            local_training_cache.terminateds,
            next_obs,
            next_terminated,
            args.num_steps,
            args.gamma,
            args.gae_lambda,
        )

        # Flatten the batch
        local_data = {
            "obs": local_training_cache.observations.reshape(
                (-1,) + envs.single_observation_space.shape
            ),
            "logprobs": local_training_cache.logprobs.reshape(-1),
            "actions": local_training_cache.actions.reshape(
                (-1,) + envs.single_action_space.shape
            ),
            "advantages": advantages.reshape(-1),
            "returns": returns.reshape(-1),
            "values": local_training_cache.values.reshape(-1),
        }

        if args.share_data:
            # Gather all the tensors from all the world and reshape them
            gathered_data = fabric.all_gather(local_data)
            for k, v in gathered_data.items():
                if k == "obs":
                    gathered_data[k] = v.reshape(
                        (-1,) + envs.single_observation_space.shape
                    )
                elif k == "actions":
                    gathered_data[k] = v.reshape((-1,) + envs.single_action_space.shape)
                else:
                    gathered_data[k] = v.reshape(-1)
        else:
            gathered_data = local_data

        # Train the agent
        train(fabric, agent, optimizer, gathered_data, global_step, args)
        fabric.log(
            "Time/step_per_second",
            int(global_step / (time.time() - start_time)),
            global_step,
        )
    return


def main(args: argparse.Namespace):
    run_name = f"{args.env_id}_{args.exp_name}_{args.seed}_{int(time.time())}"
    logger = TensorBoardLogger(
        root_dir=os.path.join(
            "logs", "fabric_logs", datetime.today().strftime("%Y-%m-%d_%H-%M-%S")
        ),
        name=run_name,
    )

    fabric = setup_fabric(args, logger)
    envs = setup_environment(args, fabric)
    agent, optimizer = setup_agent_and_optimizer(args, envs, fabric)
    # Local training cache
    training_cache = initialize_training_cache(args, envs, fabric.device)
    training_loop(args, training_cache, envs, agent, optimizer, fabric)

    envs.close()
    if fabric.is_global_zero:
        test(agent.module, fabric.device, fabric.logger.experiment, args)


if __name__ == "__main__":
    args = parse_args()
    main(args)
