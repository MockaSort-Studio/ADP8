import torch
import torch.nn.functional as F
from torch import Tensor
from torchmetrics import MeanMetric


class PPOLoss:
    def __init__(
        self,
        vf_coef: float,
        ent_coef: float,
        clip_coef: float,
        clip_vloss: bool,
        normalize_advantages: bool,
        **torchmetrics_kwargs
    ):
        self.vf_coef = vf_coef
        self.ent_coef = ent_coef
        self.clip_coef = clip_coef
        self.clip_vloss = clip_vloss
        self.normalize_advantages = normalize_advantages

        self.avg_pg_loss = MeanMetric(**torchmetrics_kwargs)
        self.avg_value_loss = MeanMetric(**torchmetrics_kwargs)
        self.avg_ent_loss = MeanMetric(**torchmetrics_kwargs)

    def compute_losses(
        self, batch, entropy, newvalue, ratio
    ) -> tuple[Tensor, Tensor, Tensor]:
        # Policy loss
        advantages = batch["advantages"]
        values = batch["values"]
        returns = batch["returns"]

        if self.normalize_advantages:
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        pg_loss = self.policy_loss(advantages, ratio, self.clip_coef)

        # Value loss
        v_loss = self.value_loss(
            newvalue,
            values,
            returns,
            self.clip_coef,
            self.clip_vloss,
            self.vf_coef,
        )

        # Entropy loss
        ent_loss = self.entropy_loss(entropy, self.ent_coef)

        # Update metrics
        self.avg_pg_loss(pg_loss)
        self.avg_value_loss(v_loss)
        self.avg_ent_loss(ent_loss)
        # Overall loss
        return pg_loss + ent_loss + v_loss

    def policy_loss(
        self, advantages: Tensor, ratio: Tensor, clip_coef: float
    ) -> Tensor:
        pg_loss1 = -advantages * ratio
        pg_loss2 = -advantages * torch.clamp(ratio, 1 - clip_coef, 1 + clip_coef)
        return torch.max(pg_loss1, pg_loss2).mean()

    def value_loss(
        self,
        new_values: Tensor,
        old_values: Tensor,
        returns: Tensor,
        clip_coef: float,
        clip_vloss: bool,
        vf_coef: float,
    ) -> Tensor:
        new_values = new_values.view(-1)
        if not clip_vloss:
            values_pred = new_values
        else:
            values_pred = old_values + torch.clamp(
                new_values - old_values, -clip_coef, clip_coef
            )
        return vf_coef * F.mse_loss(values_pred, returns)

    def entropy_loss(self, entropy: Tensor, ent_coef: float) -> Tensor:
        return -entropy.mean() * ent_coef

    def log_losses_metrics(self) -> dict[str, float]:
        loss_dict = {
            "Loss/policy_loss": self.avg_pg_loss.compute(),
            "Loss/value_loss": self.avg_value_loss.compute(),
            "Loss/entropy_loss": self.avg_ent_loss.compute(),
        }
        return loss_dict

    def reset_losses_metrics(self):
        self.avg_pg_loss.reset()
        self.avg_value_loss.reset()
        self.avg_ent_loss.reset()
