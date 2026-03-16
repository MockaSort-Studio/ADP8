load("@rules_python//python:defs.bzl", "py_runtime", "py_runtime_pair")
load("@rules_shell//shell:sh_binary.bzl", "sh_binary")
load("//tools:tools.bzl", "create_aliases", "create_compile_commands")

package(
    default_visibility = ["//visibility:public"],
)

create_aliases()

# Downloads the hermetic clang 19 toolchain (if not cached) and installs
# .bin/clangd and .bin/clang-format in the workspace root.
# Run once after container creation: bazel run //:install_clangd
sh_binary(
    name = "install_clangd",
    srcs = ["//tools:install_clangd.sh"],
    data = [
        "@llvm_toolchain//:clang-format",
        "@llvm_toolchain//:clangd",
    ],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)

create_compile_commands(
    name = "compile_cc",
    tags = [
        "manual",
        "no-remote",
    ],
    targets = {
        "//...": "",
        "@fastdds": "",
    },
)

# Define exactly how the interpreter is accessed
py_runtime(
    name = "hermetic_python_3",
    files = ["@python_3_14//:files"],
    # Points to the binary inside the downloaded hermetic repository
    interpreter = "@python_3_14//:python3",
    python_version = "PY3",
    # If the system has NO 'env' at all, you can use a hard path
    # OR an empty string to let Bazel try to resolve it relatively.
    stub_shebang = "#!/bin/sh",
)

py_runtime_pair(
    name = "hermetic_py_pair",
    py3_runtime = ":hermetic_python_3",
)

toolchain(
    name = "hermetic_toolchain",
    toolchain = ":hermetic_py_pair",
    toolchain_type = "@bazel_tools//tools/python:toolchain_type",
)
