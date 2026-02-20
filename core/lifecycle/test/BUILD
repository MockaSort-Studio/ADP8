load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_test(
    name = "execution_engine_tests",
    srcs = ["execution_engine_tests.cpp"],
    deps = [
        "//core/lifecycle",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
