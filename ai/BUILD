load("@pip_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "main",
    srcs = [
        "main.py",
    ],
    data = [
        "//ai/loss:ppo_loss",
        "//ai/models:agent",
        "//ai/parameters:examples",
        "//ai/trainers",
        "//simulation:model_mujoco",
    ],
    deps = [
        requirement("lightning"),
        requirement("torch"),
        requirement("torchmetrics"),
        requirement("gymnasium"),
        #gym deps
        requirement("moviepy"),
        requirement("pygame"),
        "//ai/core",
        "//ai/env:env_registrations",
        "//ai/utils",
        "//ai/utils:module_loader",
    ],
)
