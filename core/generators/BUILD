load("@core_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "generators",
    srcs = [
        "generators.py",
    ],
    deps = [
        requirement("jinja2"),
        "//core/generators:gen_utils",
    ],
)

py_library(
    name = "gen_utils",
    srcs = [
        "gen_data_models.py",
        "gen_utils.py",
    ],
    deps = [
        requirement("pydantic"),
        requirement("pyyaml"),
        requirement("idl_parser"),
    ],
)

filegroup(
    name = "jinja_templates",
    srcs = glob(["templates/*.jinja"]),
)
