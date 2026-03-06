load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")
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
    name = "py_mock_channel",
    srcs = ["py_mock_channel.cpp"],
    deps = [
        ":chatter_ports",
        "//core/communication:dds",
        "//simulation/py_dds_talker:py_dds_bridge",
    ],
)

py_library(
    name = "py_mock_channel_lib",
    data = [":py_mock_channel"],
    imports = ["."],
)

# py_test(
#     name = "ddsbridge_test",
#     srcs = ["ddsbridge_test.py"],
#     deps = [
#         ":channel_message_lib",
#         "//simulation/py_dds_talker:py_dds_lib",
#         requirement("pytest"),
#     ],
# )

py_binary(
    name = "py_node",
    srcs = ["py_node.py"],
    deps = [
        ":py_mock_channel_lib",
    ],
)
