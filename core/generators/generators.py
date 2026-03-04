"""Javelina-RT C++ code generator entry point.

Dispatches to DDS ports or parameters generation based on the ``--modality``
CLI argument. Reads a YAML configuration, optionally parses IDL files for type
validation, and writes C++ headers via Jinja2 templates.
"""
import argparse
import json
import sys
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, NamedTuple

from jinja2 import Template

from core.generators.gen_data_models import (
    GeneratedHeader,
    remove_bazel_prefix_path,
)
from core.generators.gen_utils import (
    DEFAULT_NAMESPACE,
    dds_ports_from_yaml,
    dds_topic_ids_pub_sub_header_models,
    dds_topic_specs_pub_sub_header_models,
    dds_types_header_model,
    get_available_idl_types,
    parameters_header_model,
    parameterset_model_from_yaml,
)


class Modality(Enum):
    """Generation modality: DDS ports (pub/sub specs) or task parameters."""

    PORTS = "ports"
    PARAMETERS = "parameters"


class _GeneratorArgs(NamedTuple):
    """Parsed and validated CLI arguments returned by parse_arguments()."""

    modality: Modality
    yaml_cfg: str
    idls: List[str]
    outputs: Dict[str, Any]
    namespace: str


def parse_arguments() -> _GeneratorArgs:
    """Parses CLI arguments and returns a validated _GeneratorArgs.

    Exits with an error if the YAML file does not exist.

    Returns:
        _GeneratorArgs with modality, yaml path, IDL list, output paths, and namespace.
    """
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
    parser.add_argument(
        "--namespace",
        required=False,
        default=DEFAULT_NAMESPACE,
        help="C++ namespace for generated code (default: gen)",
    )
    args = parser.parse_args()

    outputs_dic = json.loads(args.outputs)

    if not Path(args.yaml).exists():
        print(f"Error: YAML file not found at {args.yaml}")
        sys.exit(1)

    return _GeneratorArgs(
        modality=Modality(args.modality),
        yaml_cfg=args.yaml,
        idls=args.idls,
        outputs=outputs_dic,
        namespace=args.namespace,
    )


def generate_header_file(
    template_path: str,
    model_instance: GeneratedHeader,
) -> None:
    """Renders a Jinja2 template with model data and writes the result to disk.

    Args:
        template_path: Path to the .jinja template file.
        model_instance: Pydantic model whose fields are passed to the template.
    """
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


def generate_ports(
    yaml_cfg: str, idls: List[str], outputs: Dict[str, Any], namespace: str
) -> None:
    """Generates the five DDS port headers from a YAML configuration.

    Produces: dds_types, pub_ids, sub_ids, pub_specs, sub_specs headers.

    Args:
        yaml_cfg: Path to the ports YAML file.
        idls: List of IDL file paths for type validation.
        outputs: Dict mapping output keys to file paths (from Bazel rule).
        namespace: C++ namespace for generated code.
    """
    available_types = get_available_idl_types(idls)
    ports_model = dds_ports_from_yaml(yaml_cfg, available_types)

    dds_types_h_model = dds_types_header_model(ports_model, outputs["dds_types"])

    sub_topic_id_h = dds_topic_ids_pub_sub_header_models(
        [sub.topic_id for sub in ports_model.subscriptions],
        outputs["subscriptions"]["ids"],
        namespace=namespace,
    )
    pub_topic_id_h = dds_topic_ids_pub_sub_header_models(
        [pub.topic_id for pub in ports_model.publications],
        outputs["publications"]["ids"],
        namespace=namespace,
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
        namespace=namespace,
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
        namespace=namespace,
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


def generate_parameters(yaml_cfg: str, outputs: Dict[str, Any], namespace: str) -> None:
    """Generates a single parameters header from a YAML configuration.

    Args:
        yaml_cfg: Path to the parameters YAML file.
        outputs: Dict with a ``"parameters"`` key pointing to the output path.
        namespace: C++ namespace for generated code.
    """
    parameters_model = parameterset_model_from_yaml(yaml_cfg)
    header_model = parameters_header_model(parameters_model, outputs["parameters"], namespace=namespace)
    generate_header_file(TEMPLATES["parameters"], header_model)

    print("Generated Parameters Header:")
    print(f"{remove_bazel_prefix_path(header_model.output_file_path)}")


def main() -> None:
    """CLI entry point. Parses arguments and dispatches to the appropriate generator."""
    args = parse_arguments()
    print("=================================================================")
    print(f"🐽 🐽 🐽 🐽 🐽 Running Javelina Generators [Modality={args.modality}]")
    if args.modality == Modality.PORTS:
        generate_ports(args.yaml_cfg, args.idls, args.outputs, namespace=args.namespace)
    if args.modality == Modality.PARAMETERS:
        generate_parameters(args.yaml_cfg, args.outputs, namespace=args.namespace)

    print("🐽 🐽 🐽 🐽 🐽 Done Oink Oink!")
    print("=================================================================")


if __name__ == "__main__":
    main()
