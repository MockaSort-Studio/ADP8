import argparse
import os
import time
from datetime import datetime

import gymnasium as gym
import torch

from ai.utils.utils import parse_args
from ai.env.cartpole_env import make_cartpole_env
from lightning.fabric import Fabric
from lightning.fabric.loggers import TensorBoardLogger
from typing import Tuple
import lightning as L
from ai.train.trainer import BaseTrainer

<<<<<<< HEAD

@dataclass
class TrainingCache:
    observations: torch.Tensor
    actions: torch.Tensor
    logprobs: torch.Tensor
    rewards: torch.Tensor
    terminateds: torch.Tensor
    values: torch.Tensor
=======
from ai.utils.module_loader import import_symbol_from_file
>>>>>>> c061c43 (trainer generalization)


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
) -> Tuple[L.LightningModule, torch.optim.Optimizer]:
    ppo_loss_sym = import_symbol_from_file("ai.loss.ppo_loss", "PPOLoss")
    agent_sym = import_symbol_from_file("ai.models.agent", "PPOLightningAgent")

    ppo_loss = ppo_loss_sym(
        vf_coef=args.vf_coef,
        ent_coef=args.ent_coef,
        clip_coef=args.clip_coef,
        clip_vloss=args.clip_vloss,
        normalize_advantages=args.normalize_advantages,
    )
    agent = agent_sym(
        envs,
        ppo_loss,
        act_fun=args.activation_function,
        ortho_init=args.ortho_init,
    )

    optimizer = agent.configure_optimizers(args.learning_rate)
    agent, optimizer = fabric.setup(agent, optimizer)

    return agent, optimizer


def build_trainer(args: argparse.Namespace, logger: TensorBoardLogger) -> BaseTrainer:
    fabric = setup_fabric(args, logger)
    envs = setup_environment(args, fabric)
    agent, optimizer = setup_agent_and_optimizer(args, envs, fabric)
    trainer = import_symbol_from_file("ai.training_algorithms.ppo_train", "ppo_train")
    return trainer(
        fabric=fabric, envs=envs, agent=agent, optimizer=optimizer, args=args
    )


def main(args: argparse.Namespace):
    run_name = f"{args.env_id}_{args.exp_name}_{args.seed}_{int(time.time())}"
    logger = TensorBoardLogger(
        root_dir=os.path.join(
            "logs", "fabric_logs", datetime.today().strftime("%Y-%m-%d_%H-%M-%S")
        ),
        name=run_name,
    )
    training_controller = build_trainer(args, logger)
    training_controller.train()


if __name__ == "__main__":
    args = parse_args()
    main(args)
