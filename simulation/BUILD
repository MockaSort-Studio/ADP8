load("@rules_uv//uv:venv.bzl", "create_venv")

filegroup(
    name = "model",
    srcs = ["robot_model/spot.xml"] + glob(["robot_model/stl/*"]),
)

# Create a virtual environment for the simulation project, so that you can debug and use Intellisense
# in your VS Code editor by adding the interpreter (venv/bin/python) to your workspace.
# You must run this target to generate the venv, and re-run it if you changed the requirements.txt file.
create_venv(
    name = "simulation_venv",
    destination_folder = "simulation/venv",
    requirements_txt = "//simulation:requirements.txt",
)
