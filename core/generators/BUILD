load("@core_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "dds_ports_generator",
    srcs = [
        "dds_ports_generator.py",
    ],
    deps = [
        requirement("jinja2"),
        "//core/generators:dds_gen_utils",
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
