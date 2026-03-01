load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//core/generators:defs.bzl", "cc_dds_components")

cc_dds_components(
    name = "test",
    idls = ["//core/communication/test:test_idl"],
    ports_yaml = "test_ports.yml",
)

cc_test(
    name = "dds_lifecycle_bench",
    srcs = ["dds_task_bench.cpp"],
    deps = [
        ":lifecycle_fixture",
        ":test_types",
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
    name = "parameters_provider_tests",
    srcs = ["parameters_provider_tests.cpp"],
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
        ":lifecycle_fixture",
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
        ":test_ports",
        "//core/lifecycle:dds",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
