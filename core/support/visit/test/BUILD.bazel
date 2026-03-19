load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "visitable",
    size = "small",
    srcs = [
        "test_visitors.hpp",
        "visit_tests.cpp",
    ],
    args = ["--gtest_color=yes"],
    deps = [
        "//core/support/visit:visitable",
        "//core/support/visit:visitor_base",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
