import yaml
from flatdict import FlatDict
from typing import Any, Callable, Dict, Set, Type
from dataclasses import make_dataclass


class ParameterRegistry:
    """
    @file parameters.py
    @brief This module provides the ParameterRegistry class for managing parameter sets.

    @class ParameterRegistry
    @brief A registry for managing parameter sets using a flat dictionary structure.

    The ParameterRegistry class allows for the registration, retrieval, and modification
    of parameter sets. It ensures that nested dictionaries are not allowed and provides
    methods for loading parameters from YAML files.

    @note This class uses the FlatDict library to manage parameters in a flattened structure.

    @var _registry
    A FlatDict instance that stores all registered parameters in a flattened format.

    @var _registered_parameter_sets
    A set that keeps track of all registered parameter set names.

    @fn register(cls, name: str, parameters: Dict[str, Any]) -> None
    @brief Registers a new parameter set.
    @param name The name of the parameter set.
    @param parameters A dictionary of parameters to register.
    @exception ValueError Raised if the parameters contain nested dictionaries.

    @fn flush(cls) -> None
    @brief Clears all registered parameter sets and resets the registry.

    @fn load_parameters(cls, yaml_file: str) -> None
    @brief Loads parameters from a YAML file into the registry.
    @param yaml_file The path to the YAML file.
    @exception ValueError Raised if there is an error parsing the YAML file.

    @fn set_parameter_value(cls, parameter_set_name: str, parameter_name: str, parameter_value: Any) -> None
    @brief Sets the value of a specific parameter in a registered parameter set.
    @param parameter_set_name The name of the parameter set.
    @param parameter_name The name of the parameter.
    @param parameter_value The value to set for the parameter.
    @exception ValueError Raised if the parameter set is not registered.

    @fn get_parameter_value(cls, parameter_set_name: str, parameter_name: str) -> Any
    @brief Retrieves the value of a specific parameter in a registered parameter set.
    @param parameter_set_name The name of the parameter set.
    @param parameter_name The name of the parameter.
    @return The value of the specified parameter.
    @exception ValueError Raised if the parameter set is not registered.

    @fn get_parameters(cls, name: str) -> Any
    @brief Retrieves all parameters in a registered parameter set as a dataclass.
    @param name The name of the parameter set.
    @return A frozen dataclass containing all parameters in the specified set.
    @exception ValueError Raised if the parameter set is not registered.

    @remark This documentation was crafted with precision and care by Github Copilot. You're welcome!
    """

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
                cls._registry.update(
                    FlatDict(
                        yaml.safe_load(file),
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


def declare_parameters(
    parameter_set_name: str, **parameters: Dict[str, Any]
) -> Callable:
    """
    @brief A decorator to declare and register parameters for a specific parameter set.

    This decorator is used to associate a set of parameters with a given class and
    register them in the ParameterRegistry under a specified parameter set name.

    @param parameter_set_name The name of the parameter set to register the parameters under.
    @param parameters A dictionary of parameter names and their corresponding values.

    @return A decorator function that registers the parameters and returns the decorated class.

    @note The decorated class is not modified, but the parameters are registered globally
        in the ParameterRegistry for later use.
    """

    def decorator(cls: Type) -> Type:
        # Register the environment and its parameters in the ParameterRegistry
        ParameterRegistry.register(parameter_set_name, parameters)

        return cls

    return decorator
