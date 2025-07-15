import argparse

import gymnasium as gym
import torch
from lightning.fabric import Fabric
from typing import Any, Callable, Dict, Type
from dataclasses import dataclass
import lightning as L
from ai.utils.utils import test
from ai.core.parameters import ParameterRegistry

_COMMON_PARAMETERS: Dict[str, Any] = {"trainer_path": ""}


def register_common_trainer_parameters() -> None:
    """
    Register default parameters for the trainer.
    This function can be called to ensure that the default parameters are set.
    """
    ParameterRegistry.register("training", _COMMON_PARAMETERS)


@dataclass
class TrainingCache:
    observations: torch.Tensor
    actions: torch.Tensor
    logprobs: torch.Tensor
    rewards: torch.Tensor
    terminateds: torch.Tensor
    values: torch.Tensor


def initialize_training_cache(
    parameters: Any,
    envs: gym.vector.SyncVectorEnv,
    device: torch.device,
) -> TrainingCache:
    return TrainingCache(
        observations=torch.zeros(
            (parameters.num_steps, envs.num_envs) + envs.single_observation_space.shape,
            device=device,
        ),
        actions=torch.zeros(
            (parameters.num_steps, envs.num_envs) + envs.single_action_space.shape,
            device=device,
        ),
        logprobs=torch.zeros((parameters.num_steps, envs.num_envs), device=device),
        rewards=torch.zeros((parameters.num_steps, envs.num_envs), device=device),
        terminateds=torch.zeros((parameters.num_steps, envs.num_envs), device=device),
        values=torch.zeros((parameters.num_steps, envs.num_envs), device=device),
    )


# Trainer to be revisited
# to be revisited the test handling
def trainer(test_func: Callable) -> Callable[[Callable], Type]:
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

            def train_impl(self, training_cache: TrainingCache) -> None:
                training_loop_func(
                    self._parameters,
                    training_cache,
                    self._envs,
                    self._agent,
                    self._optimizer,
                    self._fabric,
                )

            def train(self):
                training_cache = initialize_training_cache(
                    self._parameters, self._envs, self._fabric.device
                )
                print(training_cache.actions.shape)
                self.train_impl(training_cache)
                self._envs.close()
                self.test()

            # to be revised
            def test(self):
                if self._fabric.is_global_zero:
                    test_func(self._agent, self._envs, self._parameters, self._fabric)

        return Trainer

    return decorator
