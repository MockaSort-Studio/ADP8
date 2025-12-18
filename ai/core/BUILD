load("@pip_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "core",
    srcs = [
        "model.py",
        "parameters.py",
        "training.py",
    ],
    deps = [
        requirement("lightning"),
        requirement("torch"),
        requirement("gymnasium"),
        requirement("pyyaml"),
        requirement("flatdict"),
    ],
)
