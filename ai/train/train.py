"""
Proximal Policy Optimization (PPO) - Accelerated with Lightning Fabric

Author: Federico Belotti @belerico
Adapted from https://github.com/vwxyzjn/cleanrl/blob/master/cleanrl/ppo.py
Based on the paper: https://arxiv.org/abs/1707.06347

Requirements:
- gymnasium[box2d]>=0.27.1
- moviepy
- lightning
- torchmetrics
- tensorboard


Run it with:
    fabric run --accelerator=cpu --strategy=ddp --devices=2 train_fabric.py
"""

import argparse
import torch
from ai.models.agent import PPOLightningAgent
from torch import Tensor
from torch.utils.data import BatchSampler, DistributedSampler, RandomSampler
from lightning.fabric import Fabric


def train(
    fabric: Fabric,
    agent: PPOLightningAgent,
    optimizer: torch.optim.Optimizer,
    data: dict[str, Tensor],
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
