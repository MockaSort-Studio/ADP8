load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("@simulation_pip//:requirements.bzl", "requirement")

py_binary(
    name = "example",
    srcs = ["example.py"],
    data = [":model"],
    deps = [":spotmicro"],
)

filegroup(
    name = "model",
    srcs = ["robot_model/spot.xml"] + glob(["robot_model/stl/*"]),
)

py_library(
    name = "spotmicro",
    srcs = [
        "kinematics.py",
        "spotmicroai.py",
    ],
    deps = [
        requirement("pybullet"),
        requirement("numpy"),
        requirement("matplotlib"),
    ],
)

py_library(
    name = "simulation_core",
    srcs = [
        "simulation.py",
        "simulation_agent.py",
        "simulation_scenario.py",
    ],
    deps = [
        requirement("pybullet"),
        requirement("numpy"),
    ],
)

py_binary(
    name = "simulation_main",
    srcs = ["simulation_main.py"],
    data = [":model"],
    deps = [":simulation_core"],
)

# This rule adds a convenient way to update the requirements file.
compile_pip_requirements(
    name = "requirements",
    src = "requirements.in",
    requirements_txt = "requirements.txt",
)
