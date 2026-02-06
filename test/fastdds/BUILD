load("//test/fastdds:fastdds.bzl", "cc_fastdds_types")

cc_fastdds_types(
    name = "sorcy_porty_types",
    idl_srcs = glob(["*.idl"]),
)

cc_library(
    name = "fastdds_sorcy_porty",
    srcs = ["sorcy_porty_publisher.cpp"],
    hdrs = ["sorcy_porty_publisher.hpp"],
    deps = [":sorcy_porty_types"],
)

cc_binary(
    name = "main_test",
    srcs = ["main.cpp"],
    deps = [":sorcy_porty_types"],
)

cc_binary(
    name = "sub_test",
    srcs = ["sub_main.cpp"],
    deps = [":sorcy_porty_types"],
)
