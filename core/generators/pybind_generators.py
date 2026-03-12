"""Entry point for the pybind11 DDS binding code generator.

Dispatches on --modality:
  type   → generates a pybind11 .cpp binding for IDL struct types
  bridge → generates a pybind11 .cpp binding for a PyDDSBridge instantiation
"""

import argparse
import sys
from enum import Enum
from pathlib import Path
from typing import NamedTuple

from core.generators.gen_utils import render_template
from core.generators.pybind_gen_models import (
    BridgeBindingModel,
    TypeBindingModel,
)
from core.generators.pybind_gen_utils import (
    build_bridge_model,
    build_type_models,
)

TEMPLATES = {
    "type": "core/generators/templates/pybind_type.cpp.jinja",
    "bridge": "core/generators/templates/pybind_bridge.cpp.jinja",
}


class Modality(Enum):
    TYPE = "type"
    BRIDGE = "bridge"


class _Args(NamedTuple):
    modality: Modality
    idls: list[str]
    output: str
    yaml: str | None
    namespace: str
    ports_name: str | None
    module_name: str | None


def _parse() -> _Args:
    p = argparse.ArgumentParser(description="pybind11 DDS binding generator")
    p.add_argument("--modality", required=True, choices=["type", "bridge"])
    p.add_argument("--idls", nargs="+", default=[])
    p.add_argument("--output", required=True)
    p.add_argument("--yaml", default=None)
    p.add_argument("--namespace", default="gen")
    p.add_argument("--ports-name", default=None)
    p.add_argument("--module-name", default=None)
    args = p.parse_args()
    return _Args(
        modality=Modality(args.modality),
        idls=args.idls,
        output=args.output,
        yaml=args.yaml,
        namespace=args.namespace,
        ports_name=args.ports_name,
        module_name=args.module_name,
    )


def _generate_type(args: _Args) -> None:
    if not args.idls:
        print("Error: --idls required for modality=type", file=sys.stderr)
        sys.exit(1)
    # one IDL → one output file containing all its structs
    models = build_type_models(args.idls[0], args.output)
    if len(models) != 1:
        print(
            f"Warning: {args.idls[0]} contains {len(models)} structs; "
            "only the first is bound. Multi-struct IDLs are not yet supported.",
            file=sys.stderr,
        )
    render_template(TEMPLATES["type"], models[0].model_dump(), args.output)
    print(f"Generated: {args.output}")


def _generate_bridge(args: _Args) -> None:
    if not args.yaml:
        print("Error: --yaml required for modality=bridge", file=sys.stderr)
        sys.exit(1)
    module_name = args.module_name or Path(args.output).stem
    ports_name = args.ports_name or ""
    model = build_bridge_model(
        yaml_path=args.yaml,
        idl_paths=args.idls,
        namespace=args.namespace,
        ports_name=ports_name,
        module_name=module_name,
        output_path=args.output,
    )
    render_template(TEMPLATES["bridge"], model.model_dump(), args.output)
    print(f"Generated: {args.output}")


def main() -> None:
    print("=" * 65)
    print("🐗  Running pybind11 DDS binding generator")
    args = _parse()
    if args.modality == Modality.TYPE:
        _generate_type(args)
    elif args.modality == Modality.BRIDGE:
        _generate_bridge(args)
    print("🐗  Done")
    print("=" * 65)


if __name__ == "__main__":
    main()
