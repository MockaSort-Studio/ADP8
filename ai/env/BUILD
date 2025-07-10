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
        requirement("numpy"),
        "//ai/parameters:registry",
    ],
)

py_library(
    name = "spot_env",
    srcs = [
        "spot_env.py",
    ],
    deps = [
        requirement("gymnasium"),
        requirement("numpy"),
        ":env",
    ],
)

py_library(
    name = "env_registrations",
    srcs = [
        "env_registrations.py",
    ],
    deps = [
        requirement("gymnasium"),
        ":env",
        ":spot_env",
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
        ":env_registrations",
        "//ai/parameters:registry",
    ],
)
