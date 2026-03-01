load("//tools:tools.bzl", "create_aliases", "create_compile_commands")

package(
    default_visibility = ["//visibility:public"],
)

create_aliases()

create_compile_commands(
    name = "compile_cc",
    targets = {
        "//core/...": "",
        "@fastdds": "",
    },
)
