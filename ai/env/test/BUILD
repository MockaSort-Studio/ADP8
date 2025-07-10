load("@pip_requirements//:requirements.bzl", loss_vegas_requirement = "requirement")
load("@pip_tools//:requirements.bzl", tool_requirement = "requirement")
load("@rules_python//python:defs.bzl", "py_test")

py_test(
    name = "env_test",
    srcs = [
        "env_test.py",
    ],
    data = [
        "//simulation:model_mujoco",
    ],
    deps = [
        "//ai/env",
        "//ai/env:env_registrations",
        "//ai/parameters:registry",
        tool_requirement("pytest"),
        loss_vegas_requirement("gymnasium"),
    ],
)
