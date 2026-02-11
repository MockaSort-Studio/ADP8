import argparse

import gymnasium as gym
import torch
from lightning.fabric import Fabric
from typing import Any, Callable, Dict, Type
from dataclasses import dataclass
import lightning as L
from ai.utils.module_loader import import_symbol_from_file
from ai.core.parameters import ParameterRegistry

_COMMON_PARAMETERS: Dict[str, Any] = {
    "trainer_path": "",
    "optimizer_path": "torch.optim.Adam",
    "learning_rate": 0.001,
    "epsilon": 1e-8,
}


def register_common_trainer_parameters() -> None:
    """
    Register default parameters for the trainer.
    This function can be called to ensure that the default parameters are set.
    """
    ParameterRegistry.register("training", _COMMON_PARAMETERS)


# Trainer to be revisited
# to be revisited the test handling
def trainer() -> Callable[[Callable], Type]:
    def decorator(training_loop_func: Callable) -> Type:
        class Trainer:
            def __init__(
                self,
                fabric: Fabric,
                envs: gym.vector.SyncVectorEnv,
                agent: L.LightningModule,
                optimizer: torch.optim.Optimizer,
            ):
                self._fabric = fabric
                self._envs = envs
                self._agent = agent
                self._optimizer = optimizer
                self._parameters = ParameterRegistry.get_parameters("training")
                self._name = training_loop_func.__name__

            @property
            def name(self) -> str:
                return self._name

            # TODO to be used after centralization of engine
            def configure_optimizer(self) -> torch.optim.Optimizer:

                optimizer_type = import_symbol_from_file(
                    self._parameters.optimizer_path
                )
                return optimizer_type(
                    self._agent.parameters(),
                    lr=self._parameters.learning_rate,
                    eps=self._parameters.epsilon,
                )

            def train(self) -> None:
                training_loop_func(
                    self._parameters,
                    self._envs,
                    self._agent,
                    self._optimizer,
                    self._fabric,
                )
                self._envs.close()
                # self.test()

            # to be revised
            # def test(self):
            #     if self._fabric.is_global_zero:
            #         test_func(self._agent, self._envs, self._parameters, self._fabric)

        return Trainer

    return decorator
