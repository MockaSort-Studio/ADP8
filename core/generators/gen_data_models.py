"""Pydantic models for Javelina-RT code generator inputs and outputs.

Input-side models (TopicId, TopicSpec, ParameterEntry, ParameterSet, Ports)
represent parsed YAML data. Output-side models (GeneratedHeader and subclasses)
carry all data needed to render a Jinja2 template into a C++ header.
"""
from typing import Any, Dict, List

from pydantic import BaseModel, computed_field, model_validator


class TopicId(BaseModel):
    """C++ identifier pair for a DDS topic: human-readable name and constexpr var name."""

    name: str
    c_var_name: str


class TopicSpec(BaseModel):
    """Full specification for one DDS topic: PubSubType name, topic ID, and queue depth."""

    type: str
    topic_id: TopicId
    queue_size: int


class ParameterEntry(BaseModel):
    """A single parameter: C++ tag name, type string, and default value string."""

    name: str
    type: str
    value: str


def normalize_type(raw_type: str) -> str:
    """Converts a YAML type declaration to a C++ template argument string.

    Scalars (``"float"``) pass through unchanged. Arrays (``"float[3]"``) expand
    to comma-separated repetitions (``"float,float,float"``) for use as variadic
    template arguments.

    Args:
        raw_type: YAML type string, e.g. ``"float"`` or ``"float[3]"``.

    Returns:
        C++ template argument string.
    """
    # array are modeled as type[size] we're splitting using [
    # then if we have an array we output a comma separated list
    # e.g. float[3] => float,float,float.
    # Downstream Parameter Entry in cpp is modelled as tupla<Tag, values...>
    decomposed_type = raw_type.split("[", 1)
    type = decomposed_type[0]
    if len(decomposed_type) > 2:
        raise RuntimeError(
            "What are you doing? you're supposed to have Type[Size] for array or Type for single values"
        )
    if len(decomposed_type) == 2:
        size = int(decomposed_type[1].rstrip("]"))
        type = ", ".join([type] * size)

    return type


class ParameterSet(BaseModel):
    """Collection of ParameterEntry items read from a parameters YAML file.

    The YAML root key becomes the set name (PascalCase). Each parameter name
    is suffixed with ``"Tag"`` to form the C++ tag type name.
    """

    name: str
    params: List[ParameterEntry]

    @model_validator(mode="before")
    @classmethod
    def preprocess(cls, data: Any) -> Any:
        root_key, items = next(iter(data.items()))
        transformed = {"name": normalize_name(root_key), "params": []}
        for item in items:
            name, body = next(iter(item.items()))
            value = body["value"]
            # workaround to render bool in cpp compatible form (otherwise it gets rendered with capital letter)🐽
            if isinstance(value, bool):
                value = "true" if value else "false"
            transformed["params"].append(
                {
                    "name": f"{normalize_name(name)}Tag",
                    "type": normalize_type(body["type"]),
                    # if we get a list of values we scrape the brackets
                    "value": str(value).strip("[").rstrip("]"),
                }
            )
        return transformed


def normalize_name(raw_name: str) -> str:
    """Converts a snake_case identifier to PascalCase.

    Args:
        raw_name: Snake-case name, e.g. ``"max_speed"``.

    Returns:
        PascalCase string, e.g. ``"MaxSpeed"``.
    """
    return "".join(word.capitalize() for word in raw_name.split("_"))


class Ports(BaseModel):
    """Subscriptions and publications parsed from a ports YAML file.

    The model validator flattens the list-of-dicts YAML structure into typed
    TopicSpec entries and applies defaults (queue_size=1 for publishers).
    """

    subscriptions: List[TopicSpec]
    publications: List[TopicSpec]

    @staticmethod
    def flatten_and_set_defaults(raw_list: List[Dict]) -> List[Dict]:
        """Flat helper to turn [{name: {details}}] into [{"topic_id": {"name": name}, **details}]."""
        results = []
        for entry in raw_list:
            for name, details in entry.items():
                # defaulting missing field (publishers use)
                if "queue_size" not in details.keys():
                    details["queue_size"] = 1
                topic_id = f"{normalize_name(name)}Topic"
                ###type info class from dds gen are always <MessageName>PubSubType.hpp
                details["type"] = f"{details['type']}PubSubType"

                results.append(
                    {
                        "topic_id": {
                            "name": topic_id,
                            "c_var_name": f"k{topic_id}Name",
                        },
                        **details,
                    }
                )
        return results

    @model_validator(mode="before")
    @classmethod
    def preprocess(cls, data: Any) -> Any:
        return {
            "subscriptions": cls.flatten_and_set_defaults(
                data.get("subscriptions", [])
            ),
            "publications": cls.flatten_and_set_defaults(data.get("publications", [])),
        }


def remove_bazel_prefix_path(path: str) -> str:
    """Strips the directory prefix from a Bazel-generated path, returning just the filename.

    Generated headers are included by filename only (``#include "name.hpp"``), so
    the path prefix is irrelevant. It's codegen — we don't care about folder structure 🐽

    Args:
        path: Full path string, e.g. ``"bazel-out/.../foo.hpp"``.

    Returns:
        Filename only, e.g. ``"foo.hpp"``.
    """
    parts = path.rsplit("/", 1)
    clean_path = parts[-1]
    return clean_path


class GeneratedHeader(BaseModel):
    """Base model for all generated C++ header files.

    Provides the output path, include list, namespace, and a computed header
    guard derived from the output filename.
    """

    output_file_path: str
    includes: List[str]
    namespace: str

    @computed_field
    @property
    def header_guard(self) -> str:
        short_path = remove_bazel_prefix_path(self.output_file_path)
        return f"{short_path.replace('/', '_').split('.', 1)[0].upper()}"

    @model_validator(mode="before")
    @classmethod
    def set_defaults(cls, data: Dict[str, Any]) -> Dict[str, Any]:
        if "includes" not in data:
            data["includes"] = []
        if "namespace" not in data:
            data["namespace"] = "gen"

        return data


class IdlTypeHeader(GeneratedHeader):
    """Header model listing the FastDDS-generated type include files.

    Includes one ``*PubSubTypes.hpp`` per unique type referenced in ports.
    """

    @model_validator(mode="before")
    @classmethod
    def preprocess(cls, data: Dict[str, Any]) -> Dict[str, Any]:
        ###type header from dds gen are always <MessageNamePubSubType>s.hpp
        types_h: List[str] = [f"{sub['type']}s.hpp" for sub in data["subscriptions"]]
        types_h.extend([f"{pub['type']}s.hpp" for pub in data["publications"]])

        return {
            "output_file_path": data.get("output_file_path", ""),
            "includes": list(set(types_h)),
        }


class TopicIdHeader(GeneratedHeader):
    """Header model defining constexpr topic name string constants."""

    topic_ids: List[TopicId]


class SpecsHeader(GeneratedHeader):
    """Header model defining TopicSpec instantiations and a topic list type alias."""

    topic_specs: List[TopicSpec]
    specs_list_name: str


class ParametersHeader(GeneratedHeader):
    """Header model for the parameters.hpp output, wrapping a full ParameterSet."""

    params: ParameterSet


class TaskBaseHeader(GeneratedHeader):
    """Header model for the task-base generated header.

    Carries the tag type name, task base class name, namespace, include paths
    for subscriptions and publications, the corresponding DDS type names, and
    the output path for the rendered file.
    """

    tag_name: str
    task_base_name: str
    subs_include: str
    pubs_include: str
    subs_type: str
    pubs_type: str
    output_path: str
