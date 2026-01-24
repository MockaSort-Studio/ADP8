load("@pip_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "agent",
    srcs = [
        "ppo_agent.py",
    ],
    deps = [
        requirement("lightning"),
        requirement("torch"),
        requirement("gymnasium"),
        "//ai/core",
        "//ai/loss:ppo_loss",
    ],
)
