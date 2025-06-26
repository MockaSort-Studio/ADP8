import argparse
import math
import torch
import yaml
from typing import Any, Type, TYPE_CHECKING
from torch.utils.tensorboard import SummaryWriter
from ai.env.cartpole_env import make_cartpole_env

if TYPE_CHECKING:
    from ai.models.agent import PPOLightningAgent

from dataclasses import make_dataclass


def strtobool(val):
    """Convert a string representation of truth to true (1) or false (0).

    True values are 'y', 'yes', 't', 'true', 'on', and '1'; false values are 'n', 'no', 'f', 'false', 'off', and '0'.
    Raises ValueError if 'val' is anything else.

    Note: taken from distutils after its deprecation.

    """
    val = val.lower()
    if val in ("y", "yes", "t", "true", "on", "1"):
        return 1
    if val in ("n", "no", "f", "false", "off", "0"):
        return 0
    raise ValueError(f"invalid truth value {val!r}")


def parse_yaml(file_path: str) -> Any:
    """Parse a YAML file and return its contents as a dictionary."""
    with open(file_path, "r") as file:
        try:
            params_dict = yaml.safe_load(file)
            Parameters = make_dataclass(
                "Parameters",
                params_dict.keys(),
                frozen=True,
            )
            return Parameters(**params_dict)
        except yaml.YAMLError as exc:
            raise ValueError(f"Error parsing YAML file: {exc}")


def parse_parameters() -> Any:

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--parameters",
        type=str,
        default="",
        help="path to the YAML configuration file",
    )

    args = parser.parse_args()
    parameters = parse_yaml(args.parameters)
    return parameters


def layer_init(
    layer: torch.nn.Module,
    std: float = math.sqrt(2),
    bias_const: float = 0.0,
    ortho_init: bool = True,
):
    if ortho_init:
        torch.nn.init.orthogonal_(layer.weight, std)
        torch.nn.init.constant_(layer.bias, bias_const)
    return layer


def linear_annealing(
    optimizer: torch.optim.Optimizer, episode: int, num_episodes: int, initial_lr: float
):
    frac = 1.0 - (episode - 1.0) / num_episodes
    lrnow = frac * initial_lr
    for pg in optimizer.param_groups:
        pg["lr"] = lrnow


@torch.no_grad()
def test(
    agent: "PPOLightningAgent",
    device: torch.device,
    logger: SummaryWriter,
    args: Any,
):
    env = make_cartpole_env(
        args.env_id, args.seed, 0, args.capture_video, logger.log_dir, "test"
    )()
    step = 0
    terminated = False
    cumulative_rew = 0
    next_obs = torch.tensor(env.reset(seed=args.seed)[0], device=device)
    while not terminated:
        # Act greedly through the environment
        action = agent.model.get_greedy_action(next_obs)

        # Single environment step
        next_obs, reward, terminated, truncated, _ = env.step(action.cpu().numpy())
        terminated = terminated or truncated
        cumulative_rew += reward
        next_obs = torch.tensor(next_obs, device=device)
        step += 1
    logger.add_scalar("Test/cumulative_reward", cumulative_rew, 0)
    env.close()
