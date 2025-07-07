import pytest
import os
import sys
from typing import Generator
from ai.parameters.registry import ParameterRegistry, register_parameters


@register_parameters("example_parameters")
class ExampleParameters:
    param1: str = ""
    param2: str = ""


@pytest.fixture
def yaml_file() -> Generator[str, None, None]:
    # Create a temporary YAML file for testing
    file_name: str = "test_parameters.yaml"
    with open(file_name, "w", encoding="utf-8") as file:
        file.write(
            """
example_parameters:
  param1: value1
  param2: value2
        """
        )
    yield file_name
    if os.path.exists(file_name):
        os.remove(file_name)


def test_register_parameters() -> None:
    assert "example_parameters" in ParameterRegistry._registry
    assert ParameterRegistry._registry["example_parameters"] == ExampleParameters


def test_load_parameters(yaml_file: str) -> None:
    ParameterRegistry.load_parameters(yaml_file)
    assert "example_parameters" in ParameterRegistry._loaded_data
    assert ParameterRegistry._loaded_data["example_parameters"]["param1"] == "value1"
    assert ParameterRegistry._loaded_data["example_parameters"]["param2"] == "value2"


def test_get_parameters(yaml_file: str) -> None:
    ParameterRegistry.load_parameters(yaml_file)
    parameters: ExampleParameters = ParameterRegistry.get_parameters(
        "example_parameters"
    )
    assert parameters.param1 == "value1"
    assert parameters.param2 == "value2"


def test_get_parameters_not_registered() -> None:
    with pytest.raises(
        ValueError, match="Parameters for 'nonexistent_parameters' not registered."
    ):
        ParameterRegistry.get_parameters("nonexistent_parameters")


def test_flush_registry(yaml_file: str) -> None:
    ParameterRegistry.load_parameters(yaml_file)
    assert "example_parameters" in ParameterRegistry._loaded_data

    ParameterRegistry.flush()
    assert (
        not ParameterRegistry._loaded_data
    ), "Loaded data should be empty after flush."


def test_get_parameters_not_loaded() -> None:
    ParameterRegistry.flush()
    parameters: ExampleParameters = ParameterRegistry.get_parameters(
        "example_parameters"
    )
    assert parameters.param1 == ""
    assert parameters.param2 == ""


def test_load_parameters_invalid_yaml() -> None:
    invalid_yaml_file: str = "invalid_test_parameters.yaml"
    with open(invalid_yaml_file, "w", encoding="utf-8") as file:
        file.write("invalid_yaml: [unclosed_list")

    with pytest.raises(ValueError, match="Error parsing YAML file"):
        ParameterRegistry.load_parameters(invalid_yaml_file)

    os.remove(invalid_yaml_file)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__]))
