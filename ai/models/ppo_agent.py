import math

import gymnasium as gym
import torch
import torch.nn.functional as F
from torch import Tensor
from torch.distributions import Categorical, Normal

import lightning as L
from ai.core.model import forward_variant, lightning_model
from ai.core.parameters import declare_parameters
from ai.utils.module_loader import import_symbol_from_file
from typing import Dict, Tuple


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


@declare_parameters(
    parameter_set_name="model",
    model="",
    loss="",
    activation_function="relu",
    ortho_init=False,
    normalize_advantages=False,
    learning_rate=3e-4,
    clip_coef=0.2,
    clip_vloss=False,
    ent_coef=0.0,
    vf_coef=1.0,
)
@lightning_model
class PPOAgent(torch.nn.Module):
    def __init__(
        self,
        envs: gym.vector.SyncVectorEnv,
    ) -> None:

        act_fun = self.hyperparameters.activation_function
        ortho_init = self.hyperparameters.ortho_init
        super().__init__()
        if act_fun.lower() == "relu":
            act_fun = torch.nn.ELU()
        elif act_fun.lower() == "tanh":
            act_fun = torch.nn.Tanh()
        else:
            raise ValueError(
                "Unrecognized activation function: `act_fun` must be either `relu` or `tanh`"
            )
        loss_type = import_symbol_from_file(self.hyperparameters.loss_path)
        self.loss = loss_type(
            vf_coef=self.hyperparameters.vf_coef,
            ent_coef=self.hyperparameters.ent_coef,
            clip_coef=self.hyperparameters.clip_coef,
            clip_vloss=self.hyperparameters.clip_vloss,
            normalize_advantages=self.hyperparameters.normalize_advantages,
        )
        self.critic = torch.nn.Sequential(
            layer_init(
                torch.nn.Linear(math.prod(envs.single_observation_space.shape), 256),
                ortho_init=ortho_init,
            ),
            act_fun,
            layer_init(torch.nn.Linear(256, 256), ortho_init=ortho_init),
            act_fun,
            layer_init(torch.nn.Linear(256, 1), std=1.0, ortho_init=ortho_init),
        )
        self.actor = torch.nn.Sequential(
            layer_init(
                torch.nn.Linear(math.prod(envs.single_observation_space.shape), 256),
                ortho_init=ortho_init,
            ),
            act_fun,
            layer_init(torch.nn.Linear(256, 256), ortho_init=ortho_init),
            act_fun,
            layer_init(
                torch.nn.Linear(256, math.prod(envs.single_action_space.shape)),
                std=0.01,
                ortho_init=ortho_init,
            ),
        )

    def get_action(self, x: Tensor, action: Tensor = None) -> Dict[str, Tensor]:
        act = self.actor(x)
        std = torch.nn.Parameter(torch.ones(act.shape[-1]))
        distribution = Normal(act, act * 0.0 + std)
        if action is None:
            action = distribution.sample()
        return {
            "action": action,
            "logprobs": distribution.log_prob(action).sum(dim=-1),
            "entropy": distribution.entropy(),
        }

    @forward_variant
    def get_greedy_action(self, x: Tensor) -> Dict[str, Tensor]:
        logits = self.actor(x)
        probs = F.softmax(logits, dim=-1)
        return {"action": torch.argmax(probs, dim=-1)}

    @forward_variant
    def get_value(self, x: Tensor) -> Dict[str, Tensor]:
        return {"value": self.critic(x)}

    def get_action_and_value(
        self, x: Tensor, action: Tensor = None
    ) -> Dict[str, Tensor]:
        results = self.get_action(x, action)
        value = self.get_value(x)
        results.update({"value": value["value"]})
        return results

    def forward(self, x: Tensor, action: Tensor = None) -> Dict[str, Tensor]:
        return self.get_action_and_value(x, action)

    def training_step(self, batch: dict[str, Tensor]) -> Dict[str, Tensor]:
        # Get actions and values given the current observations
        results = self(batch["observations"], batch["actions"].long())

        logratio = results["logprobs"] - batch["logprobs"]
        ratio = logratio.exp()
        combined_loss = self.loss.compute_losses(
            batch, results["entropy"], results["value"], ratio
        )
        return {
            "loss": combined_loss,
        }

    def configure_optimizers(self) -> torch.optim.Optimizer:
        return torch.optim.Adam(
            self.parameters(), lr=self.hyperparameters.learning_rate, eps=1e-4
        )

    def on_train_epoch_end(self, global_step: int) -> None:
        # Log metrics and reset their internal state
        self.logger.log_metrics(
            self.loss.log_losses_metrics(),
            global_step,
        )
        self.loss.reset_losses_metrics()
