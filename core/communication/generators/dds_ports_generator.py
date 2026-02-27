import argparse
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

import yaml
from idl_parser.parser import IDLParser
from pydantic import BaseModel, computed_field, model_validator


class TopicId(BaseModel):
    name: str

    @computed_field
    @property
    def c_var_name(self) -> str:
        return f"k{self.name.capitalize()}TopicName"

    @model_validator(mode="before")
    @classmethod
    def preprocess(cls, data: Any) -> Any:
        data["name"] = f"{data['name'].capitalize()}Topic"
        return data


class TopicSpec(BaseModel):
    type: str
    topic_id: TopicId
    queue_size: int = 0


class Ports(BaseModel):
    subscriptions: List[TopicSpec]
    publications: List[TopicSpec]

    @staticmethod
    def _preprocess(raw_list: List[Dict]) -> List[Dict]:
        """Flat helper to turn [{name: {details}}] into [{name: name, **details}]."""
        results = []
        for entry in raw_list:
            for name, details in entry.items():
                # defaulting missing field (publishers use)
                if "queue_size" not in details.keys():
                    details["queue_size"] = 1
                results.append({"topic_id": {"name": name}, **details})
        return results

    @model_validator(mode="before")
    @classmethod
    def preprocess(cls, data: Any) -> Any:
        return {
            "subscriptions": cls._preprocess(data.get("subscriptions", [])),
            "publications": cls._preprocess(data.get("publications", [])),
        }


class GeneratedHeader(BaseModel):
    header_guard: str
    includes: List[str]
    namespace: str = "generated:"


class TopicIdHeader(GeneratedHeader):
    topic_ids: List[TopicId]


class SpecsHeader(GeneratedHeader):
    topic_specs: List[TopicSpec]


def parse_arguments() -> Tuple[str, str, List[str], List[str]]:
    parser = argparse.ArgumentParser(description="DDS Ports C++ Code Generator")

    parser.add_argument(
        "--yaml", required=True, help="Path to the pubsub YAML configuration"
    )

    parser.add_argument(
        "--output_headers",
        nargs="+",
        required=True,
        help="List of header files to be generated",
    )

    parser.add_argument("--output_dir", required=True, help="Output directory")

    parser.add_argument(
        "--idls",
        nargs="+",
        required=True,
        help="Path to IDL file path (can be multiple)",
    )

    args = parser.parse_args()

    if not Path(args.yaml).exists():
        print(f"Error: YAML file not found at {args.yaml}")
        sys.exit(1)

    print(f"🔧 Generator Output Path: {args.output_dir}")
    print(f"📄 Pub Sub Config: {args.yaml}")
    print(f"🔍 Idls Files {', '.join(args.idls)}")
    print(f"🚀 Generated Files: {', '.join(args.output_headers)}")

    return args.output_dir, args.yaml, args.idls, args.output_headers


def parse_yaml(yaml_path: str) -> None:
    with open(yaml_path, "r") as file:
        try:
            ports = yaml.safe_load(file)
            return ports
        except yaml.YAMLError as exc:
            print(f"Error parsing YAML: {exc}")


def get_available_idl_types(idl_file_paths: List[str]) -> List[str]:
    parser = IDLParser(verbose=True)
    parser.parse(idl_file_paths)
    structs = parser.global_module.to_dic()["structs"]
    return [s["name"] for s in structs]


if __name__ == "__main__":
    out_dir, yaml_cfg, idls, out_h = parse_arguments()
    raw_yaml = parse_yaml(yaml_cfg)
    print(raw_yaml)
    available_types = get_available_idl_types(idls)
    print(available_types)
    ports = Ports.model_validate(raw_yaml)
    print(ports)
