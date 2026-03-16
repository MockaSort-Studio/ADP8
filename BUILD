load("//tools:tools.bzl", "create_aliases", "create_compile_commands")
load("@rules_shell//shell:sh_binary.bzl", "sh_binary")

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
