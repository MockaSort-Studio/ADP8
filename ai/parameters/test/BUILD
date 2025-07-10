load("@pip_tools//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_test")

py_test(
    name = "registry_test",
    srcs = [
        "registry_test.py",
    ],
    deps = [
        "//ai/parameters:registry",
        requirement("pytest"),
    ],
)
