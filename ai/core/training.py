import argparse

import gymnasium as gym
import torch
from lightning.fabric import Fabric
from typing import Any, Callable, Type
from dataclasses import dataclass
import lightning as L
from ai.utils.utils import test

from abc import ABC, abstractmethod


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
            (parameters.num_steps, parameters.num_envs)
            + envs.single_observation_space.shape,
            device=device,
        ),
        actions=torch.zeros(
            (parameters.num_steps, parameters.num_envs)
            + envs.single_action_space.shape,
            device=device,
        ),
        logprobs=torch.zeros(
            (parameters.num_steps, parameters.num_envs), device=device
        ),
        rewards=torch.zeros((parameters.num_steps, parameters.num_envs), device=device),
        terminateds=torch.zeros(
            (parameters.num_steps, parameters.num_envs), device=device
        ),
        values=torch.zeros((parameters.num_steps, parameters.num_envs), device=device),
    )


# add comparison model in/out shape and env action/observation space
class BaseTrainer(ABC):
    _fabric: Fabric
    _envs: gym.vector.SyncVectorEnv
    _agent: L.LightningModule
    _optimizer: torch.optim.Optimizer
    _parameters: Any
    _name: str

    @abstractmethod
    def train_impl(self, training_cache: TrainingCache) -> None:
        pass

    def train(self):
        training_cache = initialize_training_cache(
            self._parameters, self._envs, self._fabric.device
        )
        self.train_impl(training_cache)
        self._envs.close()
        if self._fabric.is_global_zero:
            test(
                self._agent.module,
                self._fabric.device,
                self._fabric.logger.experiment,
                self._parameters,
            )


def trainer() -> Callable[[Callable], Type[BaseTrainer]]:
    def decorator(training_loop_func: Callable) -> Type[BaseTrainer]:
        class SpecializedTrainer(BaseTrainer):
            def __init__(
                self,
                fabric: Fabric,
                envs: gym.vector.SyncVectorEnv,
                agent: L.LightningModule,
                optimizer: torch.optim.Optimizer,
                parameters: Any,
            ):
                self._fabric = fabric
                self._envs = envs
                self._agent = agent
                self._optimizer = optimizer
                self._parameters = parameters
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

        return SpecializedTrainer

    return decorator
