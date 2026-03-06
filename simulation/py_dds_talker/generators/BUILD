load("@core_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "pybind_generators",
    srcs = ["pybind_generators.py"],
    deps = [
        ":pybind_gen_utils",
        requirement("jinja2"),
    ],
)

py_library(
    name = "pybind_gen_utils",
    srcs = [
        "pybind_gen_models.py",
        "pybind_gen_utils.py",
    ],
    deps = [
        "//core/generators:gen_utils",
        requirement("idl_parser"),
        requirement("pydantic"),
        requirement("pyyaml"),
    ],
)

filegroup(
    name = "jinja_templates",
    srcs = glob(["templates/*.jinja"]),
)
