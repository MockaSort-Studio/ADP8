"""Public macros for generating DDS components and task parameters.

Preferred entry points over the underlying rules — they wire dependencies
and naming conventions automatically.
"""

load("//core/generators:dds_ports_gen.bzl", _cc_dds_ports = "cc_dds_ports")
load("//core/generators:fastdds_types_gen.bzl", _cc_fastdds_types = "cc_fastdds_types")
load("//core/generators:parameters_gen.bzl", _cc_parameters = "cc_parameters")

def cc_dds_components(name, idls, ports_yaml, namespace = "gen"):
    """Generates a suite of DDS communication components including port specs and type support.

    This macro instantiates two underlying rules:
    1. [cpp] ports generator that maps YAML configurations to C++ topic specifications.
    2. [cpp] FastDDS type generator that produces PubSub types from IDL files.

    Args:
        name: A unique name for this target.
        idls: List of IDL files defining the DDS data types.
        ports_yaml: YAML configuration file defining the publications and subscriptions.
        namespace: C++ namespace for generated code (default: "gen").
    """
    _cc_fastdds_types(
        name = "{}_types".format(name),
        idl_srcs = idls,
    )

    _cc_dds_ports(
        name = "{}_ports".format(name),
        idls = idls,
        yaml_config = ports_yaml,
        namespace = namespace,
        deps = [
            ":{}_types".format(name),
            "//core/communication:dds",
            "//core/lifecycle:dds",
        ],
    )

def cc_parameters(name, yaml_parameters, namespace = "gen"):
    """Generates a C++ parameters header from a YAML parameter set definition.

    Wraps the cc_parameters rule; pulls in parameters_provider as a default dep.

    Args:
        name: A unique name for this target.
        yaml_parameters: YAML file defining the parameter set.
        namespace: C++ namespace for generated code (default: "gen").
    """
    _cc_parameters(
        name = name,
        yaml_config = yaml_parameters,
        namespace = namespace,
    )
