load("@pip_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "utils",
    srcs = [
        "utils.py",
    ],
    deps = [
        requirement("torch"),
        requirement("gymnasium"),
        requirement("tensorboard"),
        "//ai/models:agent",
    ],
)

py_binary(
    name = "tensorboard",
    srcs = [
        "tensorboard.py",
    ],
    deps = [
        requirement("tensorboard"),
    ],
)
