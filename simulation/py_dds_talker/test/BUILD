load("@pip_tools//:requirements.bzl", "requirement")
load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")
load("//core/generators:defs.bzl", "cc_dds_components")

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

# ChannelMessage pybind11 binding (future: auto-generated from IDL)
pybind_extension(
    name = "channel_message_py",
    srcs = ["channel_message_py.cpp"],
    deps = [":chatter_types"],
)

py_library(
    name = "channel_message_py_lib",
    data = [":channel_message_py"],
    imports = ["."],
)

# PyDDSBridge instantiation for the chatter test node (future: auto-generated from ports YAML)
pybind_extension(
    name = "chatter_bridge_py",
    srcs = ["chatter_bridge_py.cpp"],
    deps = [
        ":chatter_ports",
        "//simulation/py_dds_talker:py_dds_bridge",
    ],
)

py_library(
    name = "chatter_bridge_py_lib",
    data = [":chatter_bridge_py"],
    imports = ["."],
    deps = [":channel_message_py_lib"],
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
