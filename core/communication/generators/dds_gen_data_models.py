from typing import Any, Dict, List

from pydantic import BaseModel, computed_field, model_validator


class TopicId(BaseModel):
    name: str
    c_var_name: str


class TopicSpec(BaseModel):
    type: str
    topic_id: TopicId
    queue_size: int


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


# unfortunately relative path do not work for saving files since exec root is somewhere else
# to void include/header guards including also bazel-out/../bin we scrape it away by using /bin as anchor
# It's not pretty, but we do not want to add complexity (e.g. adding more input variables).
# Also bazel is pretty consistent, so we can safely leave it as it is.
# We'll revisit in future if it becomes a problem 🐽
def remove_bazel_prefix_path(path: str) -> str:
    parts = path.split("bin/", 1)
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
            "includes": types_h,
        }


class TopicIdHeader(GeneratedHeader):
    topic_ids: List[TopicId]


class SpecsHeader(GeneratedHeader):
    topic_specs: List[TopicSpec]
    specs_list_name: str
