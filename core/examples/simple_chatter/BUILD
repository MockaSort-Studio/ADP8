load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//core/generators:dds_ports_gen.bzl", "cc_dds_ports")
load("//core/generators:defs.bzl", "cc_parameters")
load("//core/generators:fastdds_types_gen.bzl", "cc_fastdds_types")

cc_fastdds_types(
    name = "chatter_types",
    idl_srcs = glob(["messages/*.idl"]),
)

# node alpha
cc_dds_ports(
    name = "node_alpha_ports",
    idls = glob(["messages/*.idl"]),
    namespace = "alpha",
    yaml_config = "node_alpha_ports.yml",
    deps = [
        ":chatter_types",
        "//core/communication:dds",
    ],
)

cc_parameters(
    name = "node_alpha_params",
    yaml_parameters = "params.yaml",
)

cc_library(
    name = "node_alpha",
    hdrs = ["node_alpha.hpp"],
    deps = [
        ":node_alpha_params",
        ":node_alpha_ports",
        "//core/lifecycle:dds",
    ],
)

# node beta
cc_dds_ports(
    name = "node_beta_ports",
    idls = glob(["messages/*.idl"]),
    namespace = "beta",
    yaml_config = "node_beta_ports.yml",
    deps = [
        ":chatter_types",
        "//core/communication:dds",
    ],
)

cc_library(
    name = "node_beta",
    hdrs = ["node_beta.hpp"],
    deps = [
        ":node_beta_ports",
        "//core/lifecycle:dds",
    ],
)

# launch both nodes
cc_binary(
    name = "simple_chatter",
    srcs = ["main.cpp"],
    deps = [
        ":node_alpha",
        ":node_beta",
        "//core/lifecycle:dds",
        "//core/support/utils",
    ],
)
