"""Model builders and YAML/IDL parsing utilities for the Javelina-RT code generator."""
from typing import Any, Dict, List

import yaml
from idl_parser.parser import IDLParser

from core.generators.gen_data_models import (
    IdlTypeHeader,
    ParameterSet,
    ParametersHeader,
    Ports,
    SpecsHeader,
    TopicId,
    TopicIdHeader,
    TopicSpec,
)

DEFAULT_NAMESPACE = "gen"


def _assert_type_exists(type_name: str, available_types: List[str]) -> None:
    """Raises RuntimeError if type_name is not in available_types.

    Args:
        type_name: IDL struct name to verify.
        available_types: List of struct names parsed from the provided IDL files.
    """
    if type_name not in available_types:
        raise RuntimeError(
            f"Type '{type_name}' not found in any IDL. "
            f"Available types: {available_types}. "
            "Check your YAML config and --idl inputs."
        )


def get_available_idl_types(idl_file_paths: List[str]) -> List[str]:
    """Parses IDL files and returns all declared struct names.

    Returns an empty list (with a warning) if parsing fails. The caller decides
    whether to proceed without type validation.

    Args:
        idl_file_paths: Paths to .idl files.

    Returns:
        List of struct names found across all IDL files.
    """
    available_types: List[str] = []
    try:
        parser = IDLParser()
        parser.parse(idl_file_paths)
        structs = parser.global_module.to_dic()["structs"]
        available_types = [s["name"] for s in structs]
    except Exception as exc:
        print(f"!!! Couldn't parse IDLS: {exc}")
        print(
            "No available IDLs List, proceeding without: No preliminary consistency checks on declared types"
        )
    return available_types


def parse_yaml(yaml_path: str) -> Dict[str, Any]:
    """Loads and returns the contents of a YAML file as a dict.

    Args:
        yaml_path: Path to the YAML file.

    Returns:
        Parsed YAML as a dict.

    Raises:
        RuntimeError: If the file cannot be parsed.
    """
    with open(yaml_path, "r") as file:
        try:
            return yaml.safe_load(file)
        except yaml.YAMLError as exc:
            raise RuntimeError(f"Failed to parse YAML '{yaml_path}': {exc}") from exc


def dds_ports_from_yaml(yaml_path: str, available_types: List[str]) -> Ports:
    """Parses a ports YAML file and returns a Ports model.

    Validates that all declared types exist in the IDL files when available_types
    is non-empty. Raises RuntimeError on unknown types.

    Args:
        yaml_path: Path to the ports YAML config.
        available_types: Parsed IDL struct names for validation; skip if empty.

    Returns:
        Populated Ports model.
    """
    yaml_dict = parse_yaml(yaml_path)
    if available_types:
        for entry in (*yaml_dict["subscriptions"], *yaml_dict["publications"]):
            _assert_type_exists(next(iter(entry.values()))["type"], available_types)
    return Ports(**yaml_dict)


def dds_types_header_model(
    ports: Ports, dds_types_header_output_path: str
) -> IdlTypeHeader:
    """Builds the IdlTypeHeader model for the dds_types.hpp output.

    Args:
        ports: Ports model providing subscription and publication type names.
        dds_types_header_output_path: Output path for the generated header.

    Returns:
        IdlTypeHeader model ready for template rendering.
    """
    raw_model: Dict[str, Any] = {
        "output_file_path": dds_types_header_output_path,
        **ports.model_dump(),
    }
    return IdlTypeHeader(**raw_model)


def dds_topic_ids_pub_sub_header_models(
    topic_ids: List[TopicId],
    dds_topic_ids_header_output_path: str,
    namespace: str = DEFAULT_NAMESPACE,
) -> TopicIdHeader:
    """Builds a TopicIdHeader model listing topic name constants.

    Args:
        topic_ids: List of TopicId entries (pub or sub side).
        dds_topic_ids_header_output_path: Output path for the generated header.
        namespace: C++ namespace for the generated constants.

    Returns:
        TopicIdHeader model ready for template rendering.
    """
    return TopicIdHeader(
        output_file_path=dds_topic_ids_header_output_path,
        topic_ids=topic_ids,
        includes=[],
        namespace=namespace,
    )


def dds_topic_specs_pub_sub_header_models(
    topic_specs: List[TopicSpec],
    dds_topic_specs_header_output_path: str,
    specs_list_name: str,
    includes: List[str],
    namespace: str = DEFAULT_NAMESPACE,
) -> SpecsHeader:
    """Builds a SpecsHeader model listing TopicSpec instantiations.

    Args:
        topic_specs: List of TopicSpec entries (pub or sub side).
        dds_topic_specs_header_output_path: Output path for the generated header.
        specs_list_name: C++ type alias name for the spec tuple (e.g. ``"Subscriptions"``).
        includes: Headers to include in the generated file.
        namespace: C++ namespace for the generated specs.

    Returns:
        SpecsHeader model ready for template rendering.
    """
    model_raw: Dict[str, Any] = {
        "output_file_path": dds_topic_specs_header_output_path,
        "topic_specs": topic_specs,
        "specs_list_name": specs_list_name,
        "includes": includes,
        "namespace": namespace,
    }

    return SpecsHeader(**model_raw)


def parameterset_model_from_yaml(yaml_path: str) -> ParameterSet:
    """Parses a parameters YAML file and returns a ParameterSet model.

    Args:
        yaml_path: Path to the parameters YAML config.

    Returns:
        Populated ParameterSet model.
    """
    return ParameterSet(**parse_yaml(yaml_path))


def parameters_header_model(
    parameter_model: ParameterSet, header_output_path: str, namespace: str = DEFAULT_NAMESPACE
) -> ParametersHeader:
    """Builds a ParametersHeader model for the parameters.hpp output.

    Args:
        parameter_model: ParameterSet model from YAML parsing.
        header_output_path: Output path for the generated header.
        namespace: C++ namespace for generated code.

    Returns:
        ParametersHeader model ready for template rendering.
    """
    model_raw: Dict[str, Any] = {
        "output_file_path": header_output_path,
        "params": parameter_model,
        "namespace": namespace,
    }
    return ParametersHeader(**model_raw)
