import yaml
from dataclasses import dataclass
from flatdict import FlatDict
from typing import Any, Dict, List, Set
from dataclasses import make_dataclass

"""
ParameterRegistry and register_parameters utility for managing and loading parameters using YAML files.

This module provides a registry system for associating parameter names with dataclass types and loading their values from YAML files. It simplifies parameter management in applications by ensuring type safety and structured data handling.

Classes:
---------
- ParameterRegistry:
    A registry for managing parameter dataclasses and loading their values from YAML files.

Decorators:
-----------
- register_parameters(name):
    A decorator for registering dataclasses with the ParameterRegistry.

Usage Example:
--------------
# Define a dataclass and register it
@register_parameters("robot_config")
class RobotConfig:
    speed: float
    battery_capacity: int

# Load parameters from a YAML file
ParameterRegistry.load_parameters("config.yaml")

# Retrieve the parameters as a dataclass instance
robot_config = ParameterRegistry.get_parameters("robot_config")
print(robot_config.speed, robot_config.battery_capacity)

------------
Documented with care by GitHub Copilot!
"""


class ParameterRegistry:
    _registry: FlatDict = FlatDict({}, delimiter=".")
    _registered_parameter_sets: Set[str] = set()

    @classmethod
    def register(cls, name: str, parameters: Dict[str, Any]) -> None:
        if any(isinstance(value, dict) for value in parameters.values()):
            raise ValueError("Nested dictionaries are not allowed in parameters.")
        cls._registered_parameter_sets.add(name)
        cls._registry.update(FlatDict({name: parameters}, delimiter="."))

    @classmethod
    def flush(cls) -> None:
        cls._registered_parameter_sets = set()
        cls._registry = FlatDict({}, delimiter=".")

    @classmethod
    def load_parameters(cls, yaml_file: str) -> None:
        with open(yaml_file, "r", encoding="utf-8") as file:
            try:
                filtered_yaml = {
                    key: value
                    for key, value in yaml.safe_load(file).items()
                    if key in cls._registered_parameter_sets
                }
                cls._registry.update(
                    FlatDict(
                        filtered_yaml,
                        delimiter=".",
                    )
                )
            except yaml.YAMLError as exc:
                raise ValueError(f"Error parsing YAML file: {exc}")

    @classmethod
    def set_parameter_value(
        cls, parameter_set_name: str, parameter_name: str, parameter_value: Any
    ) -> None:
        if parameter_set_name not in cls._registered_parameter_sets:
            raise ValueError(f"Unregistered parameter set '{parameter_set_name}'")
        key = f"{parameter_set_name}.{parameter_name}"
        cls._registry[key] = parameter_value

    @classmethod
    def get_parameter_value(cls, parameter_set_name: str, parameter_name: str) -> Any:
        if parameter_set_name not in cls._registered_parameter_sets:
            raise ValueError(f"Unregistered parameter set '{parameter_set_name}'")
        key = f"{parameter_set_name}.{parameter_name}"
        return cls._registry[key]

    @classmethod
    def get_parameters(cls, name: str) -> Any:
        if name not in cls._registered_parameter_sets:
            raise ValueError(f"Unregistered parameter set '{name}'")
        # we're looking for all entries in _registry(FlatDict) having the key starting with "name."
        matching_keys = [
            key for key in cls._registry.keys() if key.startswith(f"{name}.")
        ]

        parameters_dict = {
            key.split(f"{name}.", 1)[1]: cls._registry[key] for key in matching_keys
        }
        dataclass_type = make_dataclass(
            f"{name.capitalize()}Parameters",
            parameters_dict.keys(),
            frozen=True,
        )
        return dataclass_type(**parameters_dict)
