"""This Makes the linter happy"""

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
        deps = [":{}_types".format(name), "//core/communication:dds"],
    )

def cc_parameters(name, yaml_parameters, namespace = "gen"):
    _cc_parameters(
        name = name,
        yaml_config = yaml_parameters,
        namespace = namespace,
    )
