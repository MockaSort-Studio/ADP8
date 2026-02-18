load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//core/communication:bazel/fastdds.bzl", "cc_fastdds_types")

cc_fastdds_types(
    name = "sorcy_porty_types",
    idl_srcs = glob(["*.idl"]),
)

cc_binary(
    name = "main_test",
    srcs = ["main.cpp"],
    deps = [
        ":sorcy_porty_types",
        "//core/communication:dds",
    ],
)

cc_binary(
    name = "sub_test",
    srcs = ["sub_main.cpp"],
    deps = [
        ":sorcy_porty_types",
        "//core/communication:dds",
    ],
)
