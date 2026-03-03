from typing import Any, Dict, List

from pydantic import BaseModel, computed_field, model_validator


class TopicId(BaseModel):
    name: str
    c_var_name: str


class TopicSpec(BaseModel):
    type: str
    topic_id: TopicId
    queue_size: int


class ParameterEntry(BaseModel):
    name: str
    type: str
    value: str


def normalize_type(raw_type: str) -> str:
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
    return "".join(word.capitalize() for word in raw_name.split("_"))


class Ports(BaseModel):
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


# The upstream bazel rule is configured to generate header usable just using file name (#include "name.hpp")
# It's code gen, we don't care about pretty folder structure,
# so we scrape the path away 🐽
def remove_bazel_prefix_path(path: str) -> str:
    parts = path.rsplit("/", 1)
    clean_path = parts[-1]
    return clean_path


class GeneratedHeader(BaseModel):
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
    topic_ids: List[TopicId]


class SpecsHeader(GeneratedHeader):
    topic_specs: List[TopicSpec]
    specs_list_name: str


class ParametersHeader(GeneratedHeader):
    params: ParameterSet
