load("@pip_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "training_algorithms",
    srcs = [
        "ppo_train.py",
    ],
    deps = [
        requirement("lightning"),
        requirement("torch"),
        requirement("torchmetrics"),
        requirement("gymnasium"),
        "//ai/train",
        "//ai/utils",
    ],
)
