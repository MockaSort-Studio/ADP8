load("@pip_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "ant_env",
    srcs = [
        "ant_env.py",
    ],
    data = ["//simulation:model"],
    deps = [
        requirement("gymnasium"),
    ],
)

py_library(
    name = "cartpole_env",
    srcs = [
        "cartpole_env.py",
    ],
    deps = [
        requirement("gymnasium"),
    ],
)
