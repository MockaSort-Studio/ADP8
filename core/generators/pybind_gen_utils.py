"""Model builders for the pybind11 DDS binding generator.

Parses IDL files (via idl_parser) and ports YAML (via core.generators) into
TypeBindingModel / BridgeBindingModel instances ready for Jinja2 rendering.
"""

import re
from pathlib import Path
from typing import List

import yaml
from idl_parser.parser import IDLParser

from core.generators.gen_utils import dds_ports_from_yaml, get_available_idl_types
from core.generators.pybind_gen_models import (
    IDL_TO_CPP_TYPE,
    STRING_IDL_TYPES,
    BridgeBindingModel,
    FieldModel,
    PortBindingEntry,
    TypeBindingModel,
)


def _pascal_to_snake(name: str) -> str:
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def _type_to_module(type_name: str) -> str:
    """ChannelMessage → channel_message_py"""
    return _pascal_to_snake(type_name) + "_py"


def build_type_models(idl_path: str, output_path: str) -> List[TypeBindingModel]:
    """Returns one TypeBindingModel per struct declared in the IDL file."""
    parser = IDLParser()
    parser.parse([idl_path])
    structs = parser.global_module.to_dic()["structs"]

    idl_filename = Path(idl_path).name
    models = []
    for struct in structs:
        fields = []
        for member in struct["members"]:
            idl_type = member["type"]
            cpp_type = IDL_TO_CPP_TYPE.get(idl_type, idl_type)
            fields.append(
                FieldModel(
                    name=member["name"],
                    cpp_setter_type=cpp_type,
                    is_string=idl_type in STRING_IDL_TYPES,
                )
            )
        type_name = struct["name"]
        models.append(
            TypeBindingModel(
                type_name=type_name,
                module_name=_type_to_module(type_name),
                idl_filename=idl_filename,
                fields=fields,
                output_file_path=output_path,
            )
        )
    return models


def build_bridge_model(
    yaml_path: str,
    idl_paths: List[str],
    namespace: str,
    ports_name: str,
    module_name: str,
    output_path: str,
) -> BridgeBindingModel:
    """Builds a BridgeBindingModel from a ports YAML + IDL files.

    The YAML describes the node's own perspective directly:
    - subscriptions → get_<topic>() methods
    - publications  → push_<topic>() methods
    """
    with open(yaml_path) as f:
        raw = yaml.safe_load(f)

    available_types = get_available_idl_types(idl_paths)
    ports = dds_ports_from_yaml(yaml_path, available_types)

    # Original topic names from YAML (order matches parsed ports model).
    sub_names = [next(iter(e)) for e in raw.get("subscriptions", [])]
    pub_names = [next(iter(e)) for e in raw.get("publications", [])]

    subs: List[PortBindingEntry] = []
    for topic_name, spec in zip(sub_names, ports.subscriptions):
        type_name = spec.type.replace("PubSubType", "")
        subs.append(
            PortBindingEntry(
                py_method_suffix=topic_name,
                topic_const=spec.topic_id.c_var_name,
                type_name=type_name,
                type_module=_type_to_module(type_name),
            )
        )

    pubs: List[PortBindingEntry] = []
    for topic_name, spec in zip(pub_names, ports.publications):
        type_name = spec.type.replace("PubSubType", "")
        pubs.append(
            PortBindingEntry(
                py_method_suffix=topic_name,
                topic_const=spec.topic_id.c_var_name,
                type_name=type_name,
                type_module=_type_to_module(type_name),
            )
        )

    all_entries = subs + pubs
    type_modules = list(dict.fromkeys(e.type_module for e in all_entries))
    type_names = list(dict.fromkeys(e.type_name for e in all_entries))

    return BridgeBindingModel(
        module_name=module_name,
        namespace=namespace,
        ports_name=ports_name,
        subscriptions=subs,
        publications=pubs,
        type_modules=type_modules,
        type_names=type_names,
        yaml_filename=Path(yaml_path).name,
        output_file_path=output_path,
    )
