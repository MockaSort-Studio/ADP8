import yaml
from dataclasses import dataclass
from typing import Type, Dict, Any

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
    _registry: Dict[str, Type[Any]] = {}
    _loaded_data: Dict[str, Any] = {}

    @classmethod
    def register(cls, name: str, dataclass_type: Type[Any]) -> None:
        cls._registry[name] = dataclass_type

    @classmethod
    def flush(cls) -> None:
        cls._loaded_data = {}

    @classmethod
    def load_parameters(cls, yaml_file: str) -> None:
        with open(yaml_file, "r", encoding="utf-8") as file:
            try:
                cls._loaded_data = yaml.safe_load(file)
            except yaml.YAMLError as exc:
                raise ValueError(f"Error parsing YAML file: {exc}")

    @classmethod
    def get_parameters(cls, name: str) -> Any:
        print(cls._loaded_data)
        if name not in cls._registry:
            raise ValueError(f"Parameters for '{name}' not registered.")
        dataclass_type = cls._registry[name]
        if name not in cls._loaded_data:
            # return default params (add logging line using fabric)
            return dataclass_type()
        return dataclass_type(**cls._loaded_data[name])


def register_parameters(name: str) -> Any:
    def decorator(cls: Type[Any]) -> Type[Any]:
        cls = dataclass(cls)
        ParameterRegistry.register(name, cls)
        return cls

    return decorator
