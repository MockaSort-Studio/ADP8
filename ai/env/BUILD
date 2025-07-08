load("@pip_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "env",
    srcs = [
        "env.py",
    ],
    deps = [
        requirement("gymnasium"),
        "//ai/parameters:registry",
    ],
)

py_library(
    name = "spot_env",
    srcs = [
        "spot_env.py",
    ],
    deps = [
        ":env",
        requirement("gymnasium"),
    ],
)

py_binary(
    name = "env_test",
    srcs = [
        "env_test.py",
    ],
    data = [
        "//simulation:model_mujoco",
    ],
    deps = [
        requirement("gymnasium"),
        requirement("moviepy"),
        requirement("pygame"),
        ":spot_env",
        "//ai/parameters:registry",
        "//ai/utils:module_loader",
    ],
)
