import math

import gymnasium as gym
import torch
import torch.nn.functional as F
from torch import Tensor
from torch.distributions import Categorical

import lightning as L
from ai.loss.ppo_loss import PPOLoss
from ai.core.model import forward_variant, lightning_model

from typing import Tuple


def layer_init(
    layer: torch.nn.Module,
    std: float = math.sqrt(2),
    bias_const: float = 0.0,
    ortho_init: bool = True,
) -> torch.nn.Module:
    if ortho_init:
        torch.nn.init.orthogonal_(layer.weight, std)
        torch.nn.init.constant_(layer.bias, bias_const)
    return layer


@lightning_model
class PPOAgent(torch.nn.Module):
    def __init__(
        self,
        envs: gym.vector.SyncVectorEnv,
        loss: PPOLoss,
        act_fun: str = "relu",
        ortho_init: bool = False,
    ) -> None:
        super().__init__()
        if act_fun.lower() == "relu":
            act_fun = torch.nn.ReLU()
        elif act_fun.lower() == "tanh":
            act_fun = torch.nn.Tanh()
        else:
            raise ValueError(
                "Unrecognized activation function: `act_fun` must be either `relu` or `tanh`"
            )
        self.loss = loss
        self.critic = torch.nn.Sequential(
            layer_init(
                torch.nn.Linear(math.prod(envs.single_observation_space.shape), 64),
                ortho_init=ortho_init,
            ),
            act_fun,
            layer_init(torch.nn.Linear(64, 64), ortho_init=ortho_init),
            act_fun,
            layer_init(torch.nn.Linear(64, 1), std=1.0, ortho_init=ortho_init),
        )
        self.actor = torch.nn.Sequential(
            layer_init(
                torch.nn.Linear(math.prod(envs.single_observation_space.shape), 64),
                ortho_init=ortho_init,
            ),
            act_fun,
            layer_init(torch.nn.Linear(64, 64), ortho_init=ortho_init),
            act_fun,
            layer_init(
                torch.nn.Linear(64, envs.single_action_space.n),
                std=0.01,
                ortho_init=ortho_init,
            ),
        )

    def get_action(
        self, x: Tensor, action: Tensor = None
    ) -> tuple[Tensor, Tensor, Tensor]:
        logits = self.actor(x)
        distribution = Categorical(logits=logits)
        if action is None:
            action = distribution.sample()
        return action, distribution.log_prob(action), distribution.entropy()

    def get_greedy_action(self, x: Tensor) -> Tensor:
        logits = self.actor(x)
        probs = F.softmax(logits, dim=-1)
        return torch.argmax(probs, dim=-1)

    @forward_variant
    def get_value(self, x: Tensor) -> Tensor:
        return self.critic(x)

    def get_action_and_value(
        self, x: Tensor, action: Tensor = None
    ) -> tuple[Tensor, Tensor, Tensor, Tensor]:
        action, log_prob, entropy = self.get_action(x, action)
        value = self.get_value(x)
        return action, log_prob, entropy, value

    def forward(
        self, x: Tensor, action: Tensor = None
    ) -> tuple[Tensor, Tensor, Tensor, Tensor]:
        return self.get_action_and_value(x, action)

    def training_step(self, batch: dict[str, Tensor]) -> Tuple[Tensor, Tensor, Tensor]:
        # Get actions and values given the current observations
        _, newlogprob, entropy, newvalue = self(batch["obs"], batch["actions"].long())
        logratio = newlogprob - batch["logprobs"]
        ratio = logratio.exp()
        # Policy loss
        return self.loss.compute_losses(batch, entropy, newvalue, ratio)

    def configure_optimizers(self, lr: float) -> torch.optim.Optimizer:
        return torch.optim.Adam(self.parameters(), lr=lr, eps=1e-4)

    def on_train_epoch_end(self, global_step: int) -> None:
        # Log metrics and reset their internal state
        self.logger.log_metrics(
            self.loss.log_losses_metrics(),
            global_step,
        )
        self.loss.reset_losses_metrics()
