load("@bazel_tools//tools/cpp:toolchain_utils.bzl", "find_cpp_toolchain")
load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")

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
    args.add("--yaml", ctx.file.yaml_config.path)
    args.add("--outputs", json.encode(headers))
    args.add_all("--idl", ctx.files.idls)

    ctx.actions.run(
        outputs = outputs,
        inputs = depset(
            [ctx.file.yaml_config] + ctx.files.idls + ctx.files._templates,
            transitive = [dep[DefaultInfo].files for dep in ctx.attr.deps],
        ),
        executable = ctx.executable._generator,
        arguments = [args],
        mnemonic = "DDSPortsGen",
        progress_message = "Generating DDS {} Ports".format(ctx.label.name),
    )

    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    compilation_context, compilation_outputs = cc_common.compile(
        name = ctx.label.name,
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        public_hdrs = outputs,
        compilation_contexts = [dep[CcInfo].compilation_context for dep in ctx.attr.deps],
    )

    linking_context, _ = cc_common.create_linking_context_from_compilation_outputs(
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        compilation_outputs = compilation_outputs,
        linking_contexts = [dep[CcInfo].linking_context for dep in ctx.attr.deps],
        name = ctx.label.name,
    )

    return [
        DefaultInfo(files = depset(outputs)),
        CcInfo(
            compilation_context = compilation_context,
            linking_context = linking_context,
        ),
    ]

cc_dds_ports = rule(
    implementation = _dds_ports_impl,
    attrs = {
        "yaml_config": attr.label(allow_single_file = [".yaml", ".yml"]),
        "idls": attr.label_list(allow_files = [".idl"]),
        "_generator": attr.label(
            default = Label("//core/communication/generators:dds_ports_generator"),
            executable = True,
            cfg = "exec",
        ),
        "_templates": attr.label(
            default = Label("//core/communication/generators:dds_ports_jinja_templates"),
            allow_files = [".jinja"],
        ),
        "_cc_toolchain": attr.label(default = Label("@bazel_tools//tools/cpp:current_cc_toolchain")),
        "deps": attr.label_list(providers = [CcInfo]),
    },
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
)
