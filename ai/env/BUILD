load("@pip_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "env",
    srcs = [
        "env.py",
    ],
    deps = [
        requirement("gymnasium"),
    ],
)
