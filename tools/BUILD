load("@aspect_rules_lint//format:defs.bzl", "format_multirun")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("@rules_shell//shell:sh_binary.bzl", "sh_binary")

package(default_visibility = ["//visibility:public"])

exports_files(["fixup_compile_commands_transient.sh.in"])

sh_binary(
    name = "clang_formatting",
    srcs = ["clang_format_wrap.sh"],
)

sh_binary(
    name = "shell_formatting",
    srcs = ["shell_format_wrap.sh"],
)

format_multirun(
    name = "format",
    cc = ":clang_formatting",
    #TODO python = "@aspect_rules_lint//format:ruff",
    shell = ":shell_formatting",
)
