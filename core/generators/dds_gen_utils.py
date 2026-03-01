from typing import Any, Dict, List

import yaml
from idl_parser.parser import IDLParser

from core.generators.dds_gen_data_models import (
    IdlTypeHeader,
    Ports,
    SpecsHeader,
    TopicId,
    TopicIdHeader,
    TopicSpec,
)


def type_exists(type: str, type_list: List[str]) -> bool:
    exists = type in type_list
    if not exists:
        print(f"{type} not found - Discarding port")
    return exists


def get_available_idl_types(idl_file_paths: List[str]) -> List[str]:
    parser = IDLParser()
    parser.parse(idl_file_paths)
    structs = parser.global_module.to_dic()["structs"]
    return [s["name"] for s in structs]


def dds_ports_from_yaml(yaml_path: str, available_types: List[str]) -> Ports:
    with open(yaml_path, "r") as file:
        try:
            yaml_dict = yaml.safe_load(file)
        except yaml.YAMLError as exc:
            print(f"Error parsing YAML: {exc}")
        raw_ports: Dict[str, Any] = {
            "subscriptions": [
                sub
                for sub in yaml_dict["subscriptions"]
                if type_exists(next(iter(sub.values()))["type"], available_types)
            ],
            "publications": [
                pub
                for pub in yaml_dict["publications"]
                if type_exists(next(iter(pub.values()))["type"], available_types)
            ],
        }
    return Ports(**raw_ports)


def dds_types_header_model(
    ports: Ports, dds_types_header_output_path: str
) -> IdlTypeHeader:
    raw_model: Dict[str, Any] = {
        "output_file_path": dds_types_header_output_path,
        **ports.model_dump(),
    }
    return IdlTypeHeader(**raw_model)


def dds_topic_ids_pub_sub_header_models(
    topic_ids: List[TopicId],
    dds_topic_ids_header_output_path: str,
) -> TopicIdHeader:

    return TopicIdHeader(
        output_file_path=dds_topic_ids_header_output_path,
        topic_ids=topic_ids,
        includes=[],
        namespace="gen",
    )


def dds_topic_specs_pub_sub_header_models(
    topic_specs: List[TopicSpec],
    dds_topic_specs_header_output_path: str,
    specs_list_name: str,
    includes: List[str],
) -> SpecsHeader:
    model_raw: Dict[str, Any] = {
        "output_file_path": dds_topic_specs_header_output_path,
        "topic_specs": topic_specs,
        "specs_list_name": specs_list_name,
        "includes": includes,
    }

    return SpecsHeader(**model_raw)
