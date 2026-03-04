load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//core/generators:dds_ports_gen.bzl", "cc_dds_ports")
load("//core/generators:fastdds_types_gen.bzl", "cc_fastdds_types")

cc_fastdds_types(
    name = "chatter_types",
    idl_srcs = glob(["messages/*.idl"]),
)

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
