load("@pip_tools//:requirements.bzl", tool_requirement = "requirement")
load("@rules_python//python:defs.bzl", "py_test")

py_test(
    name = "dds_gen_utils_tests",
    srcs = [
        "dds_gen_data_tests.py",
        "dds_gen_utils_tests.py",
    ],
    deps = [
        "//core/communication/generators:dds_gen_utils",
        tool_requirement("pytest"),
    ],
)
