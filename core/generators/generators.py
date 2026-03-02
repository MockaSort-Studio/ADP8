import argparse
import json
import sys
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Tuple

from jinja2 import Template

from core.generators.gen_data_models import (
    GeneratedHeader,
    remove_bazel_prefix_path,
)
from core.generators.gen_utils import (
    dds_ports_from_yaml,
    dds_topic_ids_pub_sub_header_models,
    dds_topic_specs_pub_sub_header_models,
    dds_types_header_model,
    get_available_idl_types,
    parameters_header_model,
    parameterset_model_from_yaml,
)


class Modality(Enum):
    PORTS = "ports"
    PARAMETERS = "parameters"


def parse_arguments() -> Tuple[Modality, str, List[str], Dict[str, Any]]:
    parser = argparse.ArgumentParser(description="Javelina-rt C++ Code Generator")

    parser.add_argument(
        "--yaml", required=True, help="Path to the pubsub YAML configuration"
    )

    parser.add_argument(
        "--outputs",
        required=True,
        help="Json dictionary holding output file paths",
    )

    parser.add_argument(
        "--idls",
        nargs="+",
        required=False,
        default=[],
        help="Path to IDL file path (can be multiple)",
    )
    parser.add_argument(
        "--modality",
        required=True,
        help="Modality for Javelina Generators [ports,parameters]",
    )
    args = parser.parse_args()

    outputs_dic = json.loads(args.outputs)

    if not Path(args.yaml).exists():
        print(f"Error: YAML file not found at {args.yaml}")
        sys.exit(1)

    return Modality(args.modality), args.yaml, args.idls, outputs_dic


def generate_header_file(
    template_path: str,
    model_instance: GeneratedHeader,
) -> None:

    template_content = Path(template_path).read_text()

    template = Template(template_content, trim_blocks=True, lstrip_blocks=True)

    rendered_content = template.render(model_instance.model_dump())

    output_file = Path(model_instance.output_file_path)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(rendered_content)


TEMPLATES: Dict[str, str] = {
    "ids": "core/generators/templates/dds_topic_ids.hpp.jinja",
    "specs": "core/generators/templates/dds_specs.hpp.jinja",
    "types": "core/generators/templates/dds_types.hpp.jinja",
    "parameters": "core/generators/templates/parameters.hpp.jinja",
}


def generate_ports(yaml_cfg: str, idls: List[str], outputs: Dict[str, Any]) -> None:
    available_types = get_available_idl_types(idls)
    ports_model = dds_ports_from_yaml(yaml_cfg, available_types)

    dds_types_h_model = dds_types_header_model(ports_model, outputs["dds_types"])

    sub_topic_id_h = dds_topic_ids_pub_sub_header_models(
        [sub.topic_id for sub in ports_model.subscriptions],
        outputs["subscriptions"]["ids"],
    )
    pub_topic_id_h = dds_topic_ids_pub_sub_header_models(
        [pub.topic_id for pub in ports_model.publications],
        outputs["publications"]["ids"],
    )

    subs_specs_includes = [
        remove_bazel_prefix_path(outputs["subscriptions"]["ids"]),
        remove_bazel_prefix_path(outputs["dds_types"]),
    ]
    sub_specs_h = dds_topic_specs_pub_sub_header_models(
        ports_model.subscriptions,
        outputs["subscriptions"]["specs"],
        "Subscriptions",
        subs_specs_includes,
    )

    pubs_specs_includes = [
        remove_bazel_prefix_path(outputs["publications"]["ids"]),
        remove_bazel_prefix_path(outputs["dds_types"]),
    ]
    pub_specs_h = dds_topic_specs_pub_sub_header_models(
        ports_model.publications,
        outputs["publications"]["specs"],
        "Publications",
        pubs_specs_includes,
    )

    generate_header_file(TEMPLATES["types"], dds_types_h_model)
    generate_header_file(TEMPLATES["ids"], sub_topic_id_h)
    generate_header_file(TEMPLATES["ids"], pub_topic_id_h)
    generate_header_file(TEMPLATES["specs"], sub_specs_h)
    generate_header_file(TEMPLATES["specs"], pub_specs_h)

    print("Generated Ports Headers:")
    print(f"{remove_bazel_prefix_path(dds_types_h_model.output_file_path)}")
    print(f"{remove_bazel_prefix_path(sub_topic_id_h.output_file_path)}")
    print(f"{remove_bazel_prefix_path(pub_topic_id_h.output_file_path)}")
    print(f"{remove_bazel_prefix_path(sub_specs_h.output_file_path)}")
    print(f"{remove_bazel_prefix_path(pub_specs_h.output_file_path)}")


def generate_parameters(yaml_cfg: str, outputs: Dict[str, Any]) -> None:
    parameters_model = parameterset_model_from_yaml(yaml_cfg)
    header_model = parameters_header_model(parameters_model, outputs["parameters"])
    generate_header_file(TEMPLATES["parameters"], header_model)

    print("Generated Parameters Header:")
    print(f"{remove_bazel_prefix_path(header_model.output_file_path)}")


def main() -> None:
    modality, yaml_cfg, idls, outputs = parse_arguments()
    print("=================================================================")
    print(f"🐽 🐽 🐽 🐽 🐽 Running Javelina Generators [Modality={modality}]")
    if modality == Modality.PORTS:
        generate_ports(yaml_cfg, idls, outputs)
    if modality == Modality.PARAMETERS:
        generate_parameters(yaml_cfg, outputs)

    print("🐽 🐽 🐽 🐽 🐽 Done Oink Oink!")
    print("=================================================================")


if __name__ == "__main__":
    main()
