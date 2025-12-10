load("@bazel_env.bzl", "bazel_env")

package(default_visibility = ["//:__subpackages__"])

alias(
    name = "format",
    actual = "//tools:format",
)

bazel_env(
    name = "bazel_env",
    tools = {
        # # Tools can be specified as labels.
        # "buildifier": "@buildifier_prebuilt//:buildifier",
        "format": "//tools:format",
        "clangdd": "@llvm_toolchain_llvm//:bin/clangd",
        # # Tool paths can also reference the Make variables provided by toolchains.
        # "jar": "$(JAVABASE)/bin/jar",
        # "java": "$(JAVA)",
    },
)
