load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")
load("//core/generators:cc_utils.bzl", "pack_cc_library")

def _dds_ports_impl(ctx):
    outputs = [
        ctx.actions.declare_file("{}_pub_ids.hpp".format(ctx.label.name)),
        ctx.actions.declare_file("{}_publications.hpp".format(ctx.label.name)),
        ctx.actions.declare_file("{}_sub_ids.hpp".format(ctx.label.name)),
        ctx.actions.declare_file("{}_subscriptions.hpp".format(ctx.label.name)),
        ctx.actions.declare_file("{}_dds_types.hpp".format(ctx.label.name)),
    ]

    headers = {
        "publications": {"ids": outputs[0].path, "specs": outputs[1].path},
        "subscriptions": {"ids": outputs[2].path, "specs": outputs[3].path},
        "dds_types": outputs[4].path,
    }

    args = ctx.actions.args()
    args.add("--modality", "ports")
    args.add("--yaml", ctx.file.yaml_config.path)
    args.add("--outputs", json.encode(headers))
    args.add_all("--idl", ctx.files.idls)

    ctx.actions.run(
        outputs = outputs,
        inputs = depset(
            [ctx.file.yaml_config] + ctx.files.idls + ctx.files._templates,
        ),
        executable = ctx.executable._generator,
        arguments = [args],
        mnemonic = "DDSPortsGen",
        progress_message = "Generating DDS {} Ports".format(ctx.label.name),
    )

    includes = [outputs[0].dirname] if outputs else []

    return pack_cc_library(
        ctx = ctx,
        srcs = [],
        hdrs = outputs,
        deps = ctx.attr.deps,
        includes = includes,
        linkstatic = ctx.attr.linkstatic,
    )

cc_dds_ports = rule(
    implementation = _dds_ports_impl,
    attrs = {
        "yaml_config": attr.label(allow_single_file = [".yaml", ".yml"]),
        "idls": attr.label_list(allow_files = [".idl"]),
        "_generator": attr.label(
            default = Label("//core/generators:generators"),
            executable = True,
            cfg = "exec",
        ),
        "_templates": attr.label(
            default = Label("//core/generators:jinja_templates"),
            allow_files = [".jinja"],
        ),
        "_cc_toolchain": attr.label(default = Label("@bazel_tools//tools/cpp:current_cc_toolchain")),
        "deps": attr.label_list(providers = [CcInfo]),
        "linkstatic": attr.bool(default = True),
    },
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
)
