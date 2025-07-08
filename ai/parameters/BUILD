load("@pip_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "registry",
    srcs = [
        "registry.py",
    ],
    deps = [
        requirement("pyyaml"),
        requirement("flatdict"),
    ],
)

filegroup(
    name = "parameters",
    srcs = glob(["examples/*.yaml"]),
)
