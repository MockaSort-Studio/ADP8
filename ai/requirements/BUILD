load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("@rules_uv//uv:venv.bzl", "create_venv")

# This rule adds a convenient way to update the requirements file.
compile_pip_requirements(
    name = "requirements",
    src = "requirements.in",
    requirements_txt = "requirements.txt",
)

create_venv(
    name = "venv",
    destination_folder = ".venv",
    requirements_txt = "//ai/requirements:requirements.txt",
)
