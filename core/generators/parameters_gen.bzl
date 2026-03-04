"""Bazel rule for generating a C++ parameters header from a YAML parameter set definition.

Produces a single header per target. Prefer the cc_parameters macro in defs.bzl,
which wires parameters_provider as a default dependency.
"""

load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")
load("//core/generators:cc_utils.bzl", "pack_cc_library")

def _cc_parameters_impl(ctx):
    output = ctx.actions.declare_file("{}_parameters.hpp".format(ctx.label.name))
    headers = {
        "parameters": output.path,
    }

    args = ctx.actions.args()
    args.add("--modality", "parameters")
    args.add("--yaml", ctx.file.yaml_config.path)
    args.add("--outputs", json.encode(headers))
    args.add("--namespace", ctx.attr.namespace)

    ctx.actions.run(
        outputs = [output],
        inputs = depset(
            [ctx.file.yaml_config] + ctx.files._templates,
        ),
        executable = ctx.executable._generator,
        arguments = [args],
        mnemonic = "ParametersGen",
        progress_message = "Generating {} Parameters".format(ctx.label.name),
    )

    includes = [output.dirname] if output else []

    return pack_cc_library(
        ctx = ctx,
        srcs = [],
        hdrs = [output],
        deps = ctx.attr._deps,
        includes = includes,
        linkstatic = ctx.attr.linkstatic,
    )

cc_parameters = rule(
    implementation = _cc_parameters_impl,
    attrs = {
        "yaml_config": attr.label(allow_single_file = [".yaml", ".yml"]),
        "namespace": attr.string(default = "gen"),
        "_generator": attr.label(
            default = Label("//core/generators:generators"),
            executable = True,
            cfg = "exec",
        ),
        "_deps": attr.label_list(providers = [CcInfo], default = ["//core/lifecycle:parameters_provider"]),
        "_templates": attr.label(
            default = Label("//core/generators:jinja_templates"),
            allow_files = [".jinja"],
        ),
        "_cc_toolchain": attr.label(default = Label("@bazel_tools//tools/cpp:current_cc_toolchain")),
        # "deps": attr.label_list(providers = [CcInfo]),
        "linkstatic": attr.bool(default = True),
    },
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
)
