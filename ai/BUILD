load("@pip_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "main",
    srcs = [
        "main.py",
    ],
    deps = [
        requirement("lightning"),
        requirement("gymnasium"),
    ],
)