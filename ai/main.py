import os
import time
from datetime import datetime

import gymnasium as gym
import torch

from ai.utils.utils import parse_parameters
from ai.env.env import make_env
from lightning.fabric import Fabric
from lightning.fabric.loggers import TensorBoardLogger
from typing import Any, Tuple
import lightning as L
from ai.core.training import BaseTrainer

from ai.utils.module_loader import import_symbol_from_file


def setup_fabric(parameters: Any, logger: TensorBoardLogger) -> Fabric:
    # Initialize Fabric
    fabric = Fabric(
        accelerator=parameters.accelerator,
        devices=parameters.devices,
        strategy=parameters.strategy,
        loggers=logger,
    )
    fabric.launch()
    fabric.seed_everything(parameters.seed)
    torch.backends.cudnn.deterministic = parameters.torch_deterministic

    # Log hyperparameters
    fabric.logger.experiment.add_text(
        "hyperparameters",
        "|param|value|\n|-|-|\n{}".format(
            "\n".join([f"|{key}|{value}|" for key, value in vars(parameters).items()])
        ),
    )
    return fabric


def setup_environment(parameters: Any, fabric: Fabric) -> gym.vector.SyncVectorEnv:
    rank = fabric.global_rank
    logger = fabric.logger
    # Environment setup
    envs = gym.vector.SyncVectorEnv(
        [
            make_env(
                parameters.env_id,
                parameters.seed + rank * parameters.num_envs + i,
                rank,
                parameters.capture_video,
                parameters.env_specs,
                logger.log_dir,
                "train",
            )
            for i in range(parameters.num_envs)
        ]
    )
    return envs


def setup_agent_and_optimizer(
    parameters: Any, envs: gym.vector.SyncVectorEnv, fabric: Fabric
) -> Tuple[L.LightningModule, torch.optim.Optimizer]:
    ppo_loss_sym = import_symbol_from_file(parameters.loss)
    agent_sym = import_symbol_from_file(parameters.model)

    ppo_loss = ppo_loss_sym(
        vf_coef=parameters.vf_coef,
        ent_coef=parameters.ent_coef,
        clip_coef=parameters.clip_coef,
        clip_vloss=parameters.clip_vloss,
        normalize_advantages=parameters.normalize_advantages,
    )
    agent = agent_sym(
        envs,
        ppo_loss,
        parameters.activation_function,
        parameters.ortho_init,
    )

    optimizer = agent.configure_optimizers(parameters.learning_rate)
    agent, optimizer = fabric.setup(agent, optimizer)
    for method in agent.marked_as_forward:
        agent.mark_forward_method(method)
    return agent, optimizer


def build_trainer(parameters: Any, logger: TensorBoardLogger) -> BaseTrainer:
    fabric = setup_fabric(parameters, logger)
    envs = setup_environment(parameters, fabric)
    agent, optimizer = setup_agent_and_optimizer(parameters, envs, fabric)
    trainer = import_symbol_from_file(parameters.trainer)
    return trainer(
        fabric=fabric,
        envs=envs,
        agent=agent,
        optimizer=optimizer,
        parameters=parameters,
    )


def main(parameters: Any):
    run_name = f"{parameters.env_id}_{parameters.exp_name}_{parameters.seed}_{int(time.time())}"
    logger = TensorBoardLogger(
        root_dir=os.path.join(
            "logs", "fabric_logs", datetime.today().strftime("%Y-%m-%d_%H-%M-%S")
        ),
        name=run_name,
    )
    training_controller = build_trainer(parameters, logger)
    training_controller.train()


if __name__ == "__main__":
    parameters = parse_parameters()
    main(parameters)
