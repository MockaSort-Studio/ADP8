load("@core_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("//core/communication/generators:dds_ports_gen.bzl", "dds_ports")

dds_ports(
    name = "test_ports",
    idls = glob(["test_idls/*.idl"]),
    yaml_config = "task_ports_example.yml",
)

compile_pip_requirements(
    name = "requirements",
    src = "requirements.in",
    requirements_txt = "requirements.txt",
)

py_binary(
    name = "dds_ports_generator",
    srcs = ["dds_ports_generator.py"],
    deps = [
        requirement("idl_parser"),
        requirement("pyyaml"),
        requirement("pydantic"),
    ],
)
