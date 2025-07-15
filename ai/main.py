import os
import time
from datetime import datetime

import gymnasium as gym
import torch

from ai.utils.utils import parse_arguments
from ai.env.env import build_vectorized_envs
from ai.env.env_registrations import import_envs_collection
from lightning.fabric import Fabric
from lightning.fabric.loggers import TensorBoardLogger
from typing import Any, Tuple, Type
import lightning as L
from ai.core.parameters import ParameterRegistry, declare_parameters
from ai.core.model import register_common_model_parameters
from ai.core.training import register_common_trainer_parameters

from ai.utils.module_loader import import_symbol_from_file


def setup_fabric() -> Fabric:
    parameters = ParameterRegistry.get_parameters("engine")
    run_name = f"{parameters.exp_name}_{parameters.seed}_{int(time.time())}"
    logger = TensorBoardLogger(
        root_dir=os.path.join(
            "logs", "fabric_logs", datetime.today().strftime("%Y-%m-%d_%H-%M-%S")
        ),
        name=run_name,
    )
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

    # Log hyperparameters TO BE FIXED
    fabric.logger.experiment.add_text(
        "hyperparameters",
        "|param|value|\n|-|-|\n{}".format(
            "\n".join([f"|{key}|{value}|" for key, value in vars(parameters).items()])
        ),
    )
    return fabric


def setup_agent_and_optimizer(
    envs: gym.vector.SyncVectorEnv, fabric: Fabric
) -> Tuple[L.LightningModule, torch.optim.Optimizer]:
    model_path = ParameterRegistry.get_parameter_value("model", "model_path")
    agent_sym = import_symbol_from_file(model_path)

    agent = agent_sym(envs)

    optimizer = agent.configure_optimizers()
    agent, optimizer = fabric.setup(agent, optimizer)
    for method in agent.marked_as_forward:
        agent.mark_forward_method(method)
    return agent, optimizer


def build_trainer() -> Type:
    fabric = setup_fabric()
    envs = build_vectorized_envs()  # Use the vectorized environment setup
    agent, optimizer = setup_agent_and_optimizer(envs, fabric)
    trainer_path = ParameterRegistry.get_parameter_value("training", "trainer_path")
    trainer = import_symbol_from_file(trainer_path)
    return trainer(
        fabric=fabric,
        envs=envs,
        agent=agent,
        optimizer=optimizer,
    )


@declare_parameters(
    parameter_set_name="engine",
    accelerator="auto",
    exp_name="default",
    devices=1,
    strategy="auto",
    player_on_gpu=False,
    torch_deterministic=True,
    seed=42,
)
def main():
    training_controller = build_trainer()
    training_controller.train()


if __name__ == "__main__":
    import_envs_collection()
    register_common_model_parameters()
    register_common_trainer_parameters()
    parse_arguments()
    main()
