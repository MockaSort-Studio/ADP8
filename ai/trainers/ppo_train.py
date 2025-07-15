import time

import gymnasium as gym
import torch
import torchmetrics
from ai.utils.utils import linear_annealing
from lightning.fabric import Fabric
import lightning as L
from ai.core.training import trainer, TrainingCache
from typing import Any, Dict
from torch import Tensor
from torch.utils.data import BatchSampler, DistributedSampler, RandomSampler
from ai.core.parameters import declare_parameters


@torch.no_grad()
def estimate_returns_and_advantages(
    rewards: Tensor,
    values: Tensor,
    terminateds: Tensor,
    next_terminated: Tensor,
    next_value: Tensor,
    num_steps: int,
    gamma: float,
    gae_lambda: float,
) -> tuple[Tensor, Tensor]:
    next_value = next_value.reshape(1, -1)
    advantages = torch.zeros_like(rewards)
    lastgaelam = 0
    for t in reversed(range(num_steps)):
        if t == num_steps - 1:
            nextnonterminal = torch.logical_not(next_terminated)
            nextvalues = next_value
        else:
            nextnonterminal = torch.logical_not(terminateds[t + 1])
            nextvalues = values[t + 1]
        delta = rewards[t] + gamma * nextvalues * nextnonterminal - values[t]
        advantages[t] = lastgaelam = (
            delta + gamma * gae_lambda * nextnonterminal * lastgaelam
        )
    returns = advantages + values
    return returns, advantages


def train(
    fabric: Fabric,
    agent: L.LightningModule,
    optimizer: torch.optim.Optimizer,
    data: Dict[str, Tensor],
    global_step: int,
    parameters: Any,
):
    indexes = list(range(data["obs"].shape[0]))
    if parameters.share_data:
        sampler = DistributedSampler(
            indexes,
            num_replicas=fabric.world_size,
            rank=fabric.global_rank,
            shuffle=True,
            seed=parameters.seed,
        )
    else:
        sampler = RandomSampler(indexes)
    sampler = BatchSampler(
        sampler, batch_size=parameters.per_rank_batch_size, drop_last=False
    )

    for epoch in range(parameters.update_epochs):
        if parameters.share_data:
            sampler.sampler.set_epoch(epoch)
        for batch_idxes in sampler:
            loss = agent.training_step({k: v[batch_idxes] for k, v in data.items()})
            optimizer.zero_grad(set_to_none=True)
            fabric.backward(loss)
            fabric.clip_gradients(agent, optimizer, max_norm=parameters.max_grad_norm)
            optimizer.step()
        # lets fix log and loss later
        # agent.on_train_epoch_end(global_step)


@torch.no_grad()
def test(
    agent: L.LightningModule,
    env: gym.vector.SyncVectorEnv,
    parameters: Any,
    fabric: Fabric,
):
    step = 0
    terminated = False
    cumulative_rew = 0
    device = fabric.device
    next_obs = torch.tensor(env.reset(seed=parameters.seed)[0], device=device)
    while not terminated:
        # Act greedly through the environment
        action = agent.get_greedy_action(next_obs)

        # Single environment step
        next_obs, reward, terminated, truncated, _ = env.step(action.cpu().numpy())
        terminated = torch.logical_or(
            torch.tensor(terminated), torch.tensor(truncated)
        ).any()
        cumulative_rew += reward
        next_obs = torch.tensor(next_obs, device=device)
        step += 1
    # logger.add_scalar("Test/cumulative_reward", cumulative_rew, 0)
    # env.close()


@declare_parameters(
    parameter_set_name="training",
    total_timesteps=1_000_000,
    num_steps=128,
    anneal_lr=False,
    learning_rate=3e-4,
    gamma=0.99,
    gae_lambda=0.95,
    share_data=True,
    update_epochs=4,
    max_grad_norm=0.5,
    per_rank_batch_size=64,
    seed=42,  # to be handled differently since it's a common parameter
)
@trainer(test_func=test)
def ppo_train(
    parameters: Any,
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
    num_episodes = parameters.total_timesteps // int(
        envs.num_envs * parameters.num_steps * world_size
    )

    # Player metrics
    rew_avg = torchmetrics.MeanMetric().to(device)
    ep_len_avg = torchmetrics.MeanMetric().to(device)

    # Get the first environment observation and start the optimization
    next_obs = torch.tensor(envs.reset(seed=parameters.seed)[0], device=device)
    next_terminated = torch.zeros(envs.num_envs, device=device)
    for episode in range(1, num_episodes + 1):
        # Learning rate annealing
        if parameters.anneal_lr:
            linear_annealing(optimizer, episode, num_episodes, parameters.learning_rate)
        fabric.log("Info/learning_rate", optimizer.param_groups[0]["lr"], global_step)

        for step in range(0, parameters.num_steps):
            global_step += envs.num_envs * world_size
            local_training_cache.observations[step] = next_obs
            local_training_cache.terminateds[step] = next_terminated

            # Sample an action given the observation received by the environment
            with torch.no_grad():
                action, logprob, _, value = agent.forward(next_obs)
                print(action.shape)
                local_training_cache.values[step] = value.flatten()
            print("logprob shape:", logprob.shape)
            print("action shape:", action.shape)
            print("step ", step)
            print("logprobs[step] shape:", local_training_cache.logprobs[step].shape)
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
        returns, advantages = estimate_returns_and_advantages(
            local_training_cache.rewards,
            local_training_cache.values,
            local_training_cache.terminateds,
            next_terminated,
            agent.get_value(next_obs),
            parameters.num_steps,
            parameters.gamma,
            parameters.gae_lambda,
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

        if parameters.share_data:
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
        train(fabric, agent, optimizer, gathered_data, global_step, parameters)
        fabric.log(
            "Time/step_per_second",
            int(global_step / (time.time() - start_time)),
            global_step,
        )
