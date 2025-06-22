load("@pip_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "agent",
    srcs = [
        "agent.py",
    ],
    deps = [
        requirement("lightning"),
        requirement("torch"),
        requirement("torchmetrics"),
        requirement("gymnasium"),
        "//ai/loss:ppo_loss",
    ],
)
