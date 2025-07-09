import dataclasses
import pytest
import os
import sys
from typing import Any, Dict, Generator
from ai.parameters.registry import ParameterRegistry


@pytest.fixture
def yaml_file() -> Generator[str, None, None]:
    # Create a temporary YAML file for testing
    file_name: str = "test_parameters.yaml"
    with open(file_name, "w", encoding="utf-8") as file:
        file.write(
            """
  unregistered:
    param1 : "value1"
    param2 : "value2"
  nested:
    sub1 : 1.0
    sub2 : True
    sub3 : ["oid","enac"]
        """
        )
    yield file_name
    if os.path.exists(file_name):
        os.remove(file_name)


def test_register_parameters() -> None:
    example_params: Dict[str, Any] = {"param1": "value1", "param2": "value2"}
    ParameterRegistry.register("example", example_params)
    registered = dataclasses.asdict(ParameterRegistry.get_parameters("example"))
    assert registered == example_params
    ParameterRegistry.flush()


def test_load_parameters(yaml_file: str) -> None:
    nested_params = {"sub1": 0.0, "sub2": False, "sub3": []}
    ParameterRegistry.register("nested", nested_params)
    ParameterRegistry.load_parameters(yaml_file)
    loaded = ParameterRegistry.get_parameters("nested")
    assert loaded.sub1 == 1.0
    assert loaded.sub2 == True
    assert loaded.sub3 == ["oid", "enac"]

    # checking if unresistered parameters are filtered out
    with pytest.raises(ValueError, match="Unregistered parameter set 'unregistered'"):
        ParameterRegistry.get_parameters("unregistered")


def test_set_parameter() -> None:
    registered_params = {"sub1": 0.0}
    ParameterRegistry.register("registered", registered_params)
    ParameterRegistry.set_parameter_value("registered", "sub1", 1.0)
    loaded = ParameterRegistry.get_parameters("registered")
    assert loaded.sub1 == 1.0

    # checking if get of unregistered parameters raises an error
    with pytest.raises(ValueError, match="Unregistered parameter set 'unregistered'"):
        ParameterRegistry.set_parameter_value("unregistered", "oid", "enac")


def test_get_parameters_not_registered() -> None:
    with pytest.raises(
        ValueError, match="Unregistered parameter set 'nonexistent_parameters'"
    ):
        ParameterRegistry.get_parameters("nonexistent_parameters")


def test_flush_registry() -> None:
    ParameterRegistry.register("example", {"param1": "value1", "param2": "value2"})
    assert ParameterRegistry._registry, "Registry should not be empty before flush."

    ParameterRegistry.flush()
    assert not ParameterRegistry._registry, "Loaded data should be empty after flush."


def test_load_parameters_invalid_yaml() -> None:
    invalid_yaml_file: str = "invalid_test_parameters.yaml"
    with open(invalid_yaml_file, "w", encoding="utf-8") as file:
        file.write("invalid_yaml: [unclosed_list")

    with pytest.raises(ValueError, match="Error parsing YAML file"):
        ParameterRegistry.load_parameters(invalid_yaml_file)

    os.remove(invalid_yaml_file)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__]))
