load("@core_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")

compile_pip_requirements(
    name = "requirements",
    src = "requirements.in",
    requirements_txt = "requirements.txt",
)

py_binary(
    name = "dds_tasks_ports_generator",
    srcs = ["dds_tasks_ports_generator.py"],
    data = ["//core/communication/test:test_idl"],
    deps = [requirement("idl_parser")],
)
