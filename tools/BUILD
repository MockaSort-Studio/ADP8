load("@aspect_rules_lint//format:defs.bzl", "format_multirun")
load("@buildifier_prebuilt//:rules.bzl", "buildifier")
load("@pip_lint//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

buildifier(
    name = "buildifier",
)

py_binary(
    name = "py_lint",
    srcs = ["py_lint.py"],
    deps = [requirement("black")],
)

format_multirun(
    name = "format",
    cc = "@llvm_toolchain_llvm//:bin/clang-format",
)
