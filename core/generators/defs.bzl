"""Public macros for generating DDS components and task parameters.

Preferred entry points over the underlying rules — they wire dependencies
and naming conventions automatically.
"""

load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")
load("@rules_python//python:defs.bzl", "py_library")
load("//core/generators:dds_ports_gen.bzl", _cc_dds_ports = "cc_dds_ports")
load("//core/generators:fastdds_types_gen.bzl", _cc_fastdds_types = "cc_fastdds_types")
load("//core/generators:parameters_gen.bzl", _cc_parameters = "cc_parameters")
load("//core/generators:pybind_bridge_gen.bzl", "pybind_bridge_gen")
load("//core/generators:pybind_type_gen.bzl", "pybind_type_gen")

### NOTE TODO: setting no-remote-exec for fast_dds_gen until I find a better way to find java on buildbuddy runner
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
        tags = ["no-remote-exec"],
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

### pybinds

"""Public macros for generating pybind11 DDS bindings.

Preferred entry point over the underlying rules.

Usage:

    # generates :channel_message_py (.so) + :channel_message_py_lib (py_library)
    pybind_dds_type(
        name = "channel_message_py",
        idl = "messages/ChannelMessage.idl",
        type_deps = [":chatter_types"],
    )

    # generates :chatter_bridge_py (.so) + :chatter_bridge_py_lib (py_library)
    pybind_dds_bridge(
        name = "chatter_bridge_py",
        ports_yaml = "node_cpp_ports.yml",
        idls = glob(["messages/*.idl"]),
        namespace = "cpp",
        ports_name = "chatter_ports",
        bridge_deps = [":chatter_ports", "//core/pybinds:py_dds_bridge"],
        type_lib = ":channel_message_py_lib",
    )

    # or both at once (assumes cc_dds_components(name=name) was already called)
    pybind_dds_components(
        name = "chatter",
        idls = glob(["messages/*.idl"]),
        ports_yaml = "node_cpp_ports.yml",
        namespace = "cpp",
    )
"""

def pybind_dds_type(name, idl, type_deps, visibility = None):
    """Generates a pybind11 type binding .so + py_library wrapper for one IDL file.

    Produces:
      :{name}        — pybind_extension (.so)
      :{name}_lib    — py_library wrapping the extension

    Args:
        name:      Python module name (= Bazel target name, e.g. "channel_message_py").
        idl:       Label of the .idl source file.
        type_deps: C++ deps for the pybind_extension (e.g. [":chatter_types"]).
        visibility: Bazel visibility list.
    """
    pybind_type_gen(
        name = name + "_srcs",
        idls = [idl],
        visibility = ["//visibility:private"],
    )

    pybind_extension(
        name = name,
        srcs = [":" + name + "_srcs"],
        deps = type_deps,
        visibility = visibility,
    )

    py_library(
        name = name + "_lib",
        data = [":" + name],
        imports = ["."],
        visibility = visibility,
    )

def pybind_dds_bridge(name, ports_yaml, idls, namespace, ports_name, bridge_deps, type_lib, visibility = None):
    """Generates a pybind11 PyDDSBridge binding .so + py_library wrapper.

    Produces:
      :{name}      — pybind_extension (.so)
      :{name}_lib  — py_library (depends on type_lib for cross-module type sharing)

    Args:
        name:        Python module name (e.g. "chatter_bridge_py").
        ports_yaml:  Label of the ports YAML config file.
        idls:        List of IDL file labels.
        namespace:   C++ namespace of the generated ports (e.g. "cpp").
        ports_name:  Base name of the generated ports headers (e.g. "chatter_ports").
        bridge_deps: C++ deps for the pybind_extension (e.g. port specs + py_dds_bridge).
        type_lib:    py_library target providing the type binding module.
        visibility:  Bazel visibility list.

    The ports YAML should describe the Python node's own perspective (its subscriptions
    and publications directly), not the counterpart node's perspective.
    """
    pybind_bridge_gen(
        name = name + "_srcs",
        yaml_config = ports_yaml,
        idls = idls,
        namespace = namespace,
        ports_name = ports_name,
        module_name = name,
        visibility = ["//visibility:private"],
    )

    pybind_extension(
        name = name,
        srcs = [":" + name + "_srcs"],
        deps = bridge_deps,
        visibility = visibility,
    )

    py_library(
        name = name + "_lib",
        data = [":" + name],
        imports = ["."],
        deps = [type_lib],
        visibility = visibility,
    )

def pybind_dds_components(name, idls, ports_yaml, namespace = "gen", visibility = None):
    """Generates pybind11 bindings for all types and the bridge for a DDS component set.

    Assumes cc_dds_components(name=name, ...) has already been called in the same
    BUILD file, producing :{name}_types and :{name}_ports.

    Produces (example for name="chatter"):
      :channel_message_py / :channel_message_py_lib   — per-IDL type bindings
      :chatter_bridge_py  / :chatter_bridge_py_lib     — bridge binding

    Note: currently generates one type binding per IDL file, named after the
    IDL basename (e.g. ChannelMessage.idl → channel_message_py).

    Args:
        name:       Component name matching the cc_dds_components call (e.g. "chatter").
        idls:       List of IDL file labels (same as cc_dds_components).
        ports_yaml: Ports YAML config label (same as cc_dds_components).
        namespace:  C++ namespace (same as cc_dds_components, default: "gen").
        visibility: Bazel visibility list.
    """
    type_libs = []
    for idl in idls:
        # Derive module name from IDL basename: "messages/ChannelMessage.idl" → "channel_message_py"
        idl_base = idl.split("/")[-1].replace(".idl", "")

        # PascalCase → snake_case
        snake = ""
        for i, c in enumerate(idl_base):
            if c.isupper() and i > 0:
                snake += "_"
            snake += c.lower()
        type_name = snake + "_py"

        pybind_dds_type(
            name = type_name,
            idl = idl,
            type_deps = [":{}_types".format(name)],
            visibility = visibility,
        )
        type_libs.append(":" + type_name + "_lib")

    bridge_name = name + "_bridge_py"
    pybind_dds_bridge(
        name = bridge_name,
        ports_yaml = ports_yaml,
        idls = idls,
        namespace = namespace,
        ports_name = "{}_ports".format(name),
        bridge_deps = [
            ":{}_ports".format(name),
            "//core/pybinds:py_dds_bridge",
        ],
        # for now assume single type lib; multi-IDL case will need a wrapper
        type_lib = type_libs[0],
        visibility = visibility,
    )
