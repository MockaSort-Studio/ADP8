load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("@rules_uv//uv:venv.bzl", "create_venv")
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
    visibility = ["//ai/env:__subpackages__"],
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

# Create a virtual environment for the simulation project, so that you can debug and use Intellisense
# in your VS Code editor by adding the interpreter (venv/bin/python) to your workspace.
# You must run this target to generate the venv, and re-run it if you changed the requirements.txt file.
create_venv(
    name = "simulation_venv",
    destination_folder = "simulation/venv",
    requirements_txt = "//simulation:requirements.txt",
)
