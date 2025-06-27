load("@pip_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "ppo_loss",
    srcs = [
        "ppo_loss.py",
    ],
    deps = [
        requirement("torch"),
    ],
)
