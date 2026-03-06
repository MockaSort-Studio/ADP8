load("@pip_tools//:requirements.bzl", "requirement")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_binary", "py_test")
load("//core/generators:defs.bzl", "cc_dds_components")
load("//simulation/py_dds_talker/generators:defs.bzl", "pybind_dds_bridge", "pybind_dds_type")

# cpp node
cc_dds_components(
    name = "chatter",
    idls = glob(["messages/*.idl"]),
    namespace = "cpp",
    ports_yaml = "node_cpp_ports.yml",
)

cc_library(
    name = "node_cpp",
    hdrs = ["node_cpp.hpp"],
    deps = [
        ":chatter_ports",
        "//core/lifecycle:dds",
    ],
)

cc_binary(
    name = "cpp_node",
    srcs = ["main_cpp.cpp"],
    deps = [
        ":node_cpp",
        "//core/lifecycle:dds",
        "//core/support/utils",
    ],
)

pybind_dds_type(
    name = "channel_message_py",
    idl = "messages/ChannelMessage.idl",
    type_deps = [":chatter_types"],
)

pybind_dds_bridge(
    name = "chatter_bridge_py",
    ports_yaml = "node_cpp_ports.yml",
    idls = glob(["messages/*.idl"]),
    namespace = "cpp",
    ports_name = "chatter_ports",
    bridge_deps = [
        ":chatter_ports",
        "//simulation/py_dds_talker:py_dds_bridge",
    ],
    type_lib = ":channel_message_py_lib",
)

py_test(
    name = "ddsbridge_test",
    srcs = ["ddsbridge_test.py"],
    deps = [
        ":chatter_bridge_py_lib",
        requirement("pytest"),
    ],
)

py_binary(
    name = "py_node",
    srcs = ["py_node.py"],
    deps = [":chatter_bridge_py_lib"],
)
