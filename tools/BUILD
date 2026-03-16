load("@aspect_rules_lint//format:defs.bzl", "format_multirun")
load("@rules_shell//shell:sh_binary.bzl", "sh_binary")

package(default_visibility = ["//visibility:public"])

exports_files([
    "fixup_compile_commands_transient.sh.in",
    "install_clangd.sh",
])

sh_binary(
    name = "shell_formatting",
    srcs = ["shell_format_wrap.sh"],
)

format_multirun(
    name = "format",
    cc = "@llvm_toolchain//:clang-format",
    shell = ":shell_formatting",
)
