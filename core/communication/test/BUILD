load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//core/communication:bazel/fastdds.bzl", "cc_fastdds_types")

cc_fastdds_types(
    name = "test_dds_types",
    idl_srcs = glob(["idl/*.idl"]),
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
    name = "dds_topic_test",
    srcs = ["dds_topic_test.cpp"],
    deps = [
        ":test_dds_types",
        "//core/communication:dds",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
