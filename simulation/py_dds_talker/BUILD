load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")
load("@rules_python//python:defs.bzl", "py_library")

pybind_extension(
    name = "py_dds_bridge",
    srcs = ["py_dds_bridge.cpp"],
    deps = [
        "//core/lifecycle:dds",
    ],
)

py_library(
    name = "py_dds_lib",
    data = [":py_dds_bridge"],
    imports = ["."],
    visibility = ["//visibility:public"],
)