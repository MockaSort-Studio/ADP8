load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_test(
    name = "dds_lifecycle_bench",
    srcs = ["dds_task_bench.cpp"],
    copts = [
        "-O3",
        "-march=native",
    ],
    deps = [
        ":lifecycle_fixture",
        "//core/communication/test:test_dds_types",
        "@google_benchmark//:benchmark_main",
    ],
)

cc_test(
    name = "execution_engine_tests",
    srcs = ["execution_engine_tests.cpp"],
    deps = [
        "//core/lifecycle",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "task_manager_tests",
    srcs = ["task_manager_tests.cpp"],
    deps = [
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "dds_application_tests",
    srcs = ["dds_application_tests.cpp"],
    deps = [
        ":lifecycle_fixture",
    ],
)

cc_test(
    name = "dds_task_tests",
    srcs = ["dds_task_tests.cpp"],
    deps = [
        ":lifecycle_fixture",
    ],
)

cc_test(
    name = "input_output_tests",
    srcs = ["input_output_tests.cpp"],
    deps = [
        ":lifecycle_fixture",
    ],
)

cc_library(
    name = "lifecycle_fixture",
    testonly = True,
    srcs = [],
    hdrs = ["lifecycle_fixture.hpp"],
    deps = [
        "//core/communication/generators:test_ports",
        "//core/communication/test:test_dds_types",
        "//core/lifecycle",
        "//core/lifecycle:dds",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
