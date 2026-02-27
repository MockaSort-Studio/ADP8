load("@core_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("//core/communication/generators:dds_ports_gen.bzl", "cc_dds_ports")
load("//core/communication/generators:fastdds_types_gen.bzl", "cc_fastdds_types")

package(default_visibility = ["//visibility:public"])

cc_dds_ports(
    name = "test_ports",
    idls = glob(["test_idls/*.idl"]),
    yaml_config = "task_ports_example.yml",
    deps = [
        ":test_types",
        "//core/communication:dds",
    ],
)

cc_fastdds_types(
    name = "test_types",
    idl_srcs = glob(["test_idls/*.idl"]),
)

compile_pip_requirements(
    name = "requirements",
    src = "requirements.in",
    requirements_txt = "requirements.txt",
)

py_binary(
    name = "dds_ports_generator",
    srcs = [
        "dds_ports_generator.py",
    ],
    deps = [
        requirement("jinja2"),
        "//core/communication/generators:dds_gen_utils",
    ],
)

py_library(
    name = "dds_gen_utils",
    srcs = [
        "dds_gen_data_models.py",
        "dds_gen_utils.py",
    ],
    deps = [
        requirement("pydantic"),
        requirement("pyyaml"),
        requirement("idl_parser"),
    ],
)

filegroup(
    name = "dds_ports_jinja_templates",
    srcs = glob(["templates/*.jinja"]),
)
