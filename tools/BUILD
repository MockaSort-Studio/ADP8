load("@aspect_rules_lint//format:defs.bzl", "format_multirun")
load("@buildifier_prebuilt//:rules.bzl", "buildifier")
load("@pip_tools//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")

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

compile_pip_requirements(
    name = "requirements",
    src = "requirements.in",
    requirements_txt = "requirements.txt",
)
