load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("//core/generators:dds_ports_gen.bzl", "cc_dds_ports")
load("//core/generators:fastdds_types_gen.bzl", "cc_fastdds_types")

# shared message types
cc_fastdds_types(
    name = "chatter_types",
    idl_srcs = glob(["messages/*.idl"]),
)

# cpp node
cc_dds_ports(
    name = "node_cpp_ports",
    idls = glob(["messages/*.idl"]),
    namespace = "cpp",
    yaml_config = "node_cpp_ports.yml",
    deps = [
        ":chatter_types",
        "//core/communication:dds",
    ],
)

cc_library(
    name = "node_cpp",
    hdrs = ["node_cpp.hpp"],
    deps = [
        ":node_cpp_ports",
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

# ChannelMessage pybind11 binding (future: auto-generated from IDL)
pybind_extension(
    name = "channel_message_py",
    srcs = ["channel_message_py.cpp"],
    deps = [":chatter_types"],
)

py_library(
    name = "channel_message_lib",
    data = [":channel_message_py"],
    imports = ["."],
)

# python node
py_binary(
    name = "dry_run_bridge_test",
    srcs = ["dry_run_bridge_test.py"],
    deps = [
        ":channel_message_lib",
        "//simulation/py_dds_talker:py_dds_lib",
    ],
)
