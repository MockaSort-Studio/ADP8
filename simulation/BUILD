load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("@rules_uv//uv:venv.bzl", "create_venv")
load("@simulation_pip//:requirements.bzl", "requirement")

filegroup(
    name = "model",
    srcs = ["robot_model/spot.xml"] + glob(["robot_model/stl/*"]),
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

py_library(
    name = "simulation_parameters",
    srcs = ["simulation_parameters.py"],
    data = ["simulation_parameters.yaml"],
    deps = [
        requirement("pyyaml"),
        requirement("pybullet"),
        requirement("numpy"),
    ],
)

py_library(
    name = "simulation_entity_base",
    srcs = ["simulation_entity_base.py"],
)

py_library(
    name = "simulation_runner",
    srcs = ["simulation_runner.py"],
    deps = [
        ":simulation_entity_base",
        requirement("pybullet"),
        requirement("numpy"),
    ],
)


filegroup(
    name = "spot_model",
    srcs = ["robot_model/spot.xml"] + glob(["robot_model/stl/*"]),
)

py_library(
    name = "spotmicro_sim_model",
    srcs = ["spotmicro_sim_model.py"],
    data = [":spot_model"],
    deps = [
        ":simulation_entity_base",
        requirement("pybullet"),
        requirement("numpy"),
    ],
)

py_binary(
    name = "simulation_main",
    srcs = ["simulation_main.py"],
    deps = [
        ":simulation_runner",
        ":spotmicro_sim_model",
        ":simulation_parameters",
        requirement("pybullet"),
        requirement("numpy"),
    ],
)
