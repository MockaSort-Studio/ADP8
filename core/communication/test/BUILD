load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//core/communication/generators:fastdds_types_gen.bzl", "cc_fastdds_types")

filegroup(
    name = "test_idl",
    srcs = ["idl/Test.idl"],
    visibility = ["//core:__subpackages__"],
)

cc_fastdds_types(
    name = "test_dds_types",
    idl_srcs = glob(["idl/*.idl"]),
    visibility = ["//core/lifecycle/test:__subpackages__"],
)

cc_test(
    name = "fastdds_integration_test",
    srcs = ["dds_pub_sub_test.cpp"],
    deps = [
        ":test_dds_types",
        "//core/communication:dds",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "topic_spec_tests",
    srcs = ["topic_spec_tests.cpp"],
    deps = [
        ":test_dds_types",
        "//core/communication:dds",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "dds_context_tests",
    srcs = ["dds_context_tests.cpp"],
    deps = [
        ":test_dds_types",
        "//core/communication:dds",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
