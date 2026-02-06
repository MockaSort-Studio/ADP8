load("@aspect_rules_lint//format:defs.bzl", "format_multirun")
load("@buildifier_prebuilt//:rules.bzl", "buildifier")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")

package(default_visibility = ["//visibility:public"])

buildifier(
    name = "buildifier",
)

sh_binary(
    name = "system_clang_format",
    srcs = ["system_clang_format.sh"],
)

sh_binary(
    name = "fastddsgen",
    srcs = ["fastddsgen.sh"],
    visibility = ["//visibility:public"],
)

format_multirun(
    name = "format",
    cc = ":system_clang_format",
)

compile_pip_requirements(
    name = "requirements",
    src = "requirements.in",
    requirements_txt = "requirements.txt",
)
