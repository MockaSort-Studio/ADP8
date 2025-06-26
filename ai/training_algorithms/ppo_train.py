import argparse
import time

import gymnasium as gym
import torch
import torchmetrics
from ai.utils.utils import linear_annealing
from lightning.fabric import Fabric
import lightning as L
from ai.train.trainer import trainer, TrainingCache
from typing import Dict
from torch import Tensor
from torch.utils.data import BatchSampler, DistributedSampler, RandomSampler


def train(
    fabric: Fabric,
    agent: L.LightningModule,
    optimizer: torch.optim.Optimizer,
    data: Dict[str, Tensor],
    global_step: int,
    args: argparse.Namespace,
):
    indexes = list(range(data["obs"].shape[0]))
    if args.share_data:
        sampler = DistributedSampler(
            indexes,
            num_replicas=fabric.world_size,
            rank=fabric.global_rank,
            shuffle=True,
            seed=args.seed,
        )
    else:
        sampler = RandomSampler(indexes)
    sampler = BatchSampler(
        sampler, batch_size=args.per_rank_batch_size, drop_last=False
    )

    for epoch in range(args.update_epochs):
        if args.share_data:
            sampler.sampler.set_epoch(epoch)
        for batch_idxes in sampler:
            loss = agent.training_step({k: v[batch_idxes] for k, v in data.items()})
            optimizer.zero_grad(set_to_none=True)
            fabric.backward(loss)
            fabric.clip_gradients(agent, optimizer, max_norm=args.max_grad_norm)
            optimizer.step()
        agent.on_train_epoch_end(global_step)


@trainer()
def ppo_train(
    args: argparse.Namespace,
    local_training_cache: TrainingCache,
    envs: gym.vector.SyncVectorEnv,
    agent: L.LightningModule,
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
