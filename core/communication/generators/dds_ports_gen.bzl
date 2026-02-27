load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")

def _dds_ports_impl(ctx):
    pub_ids_h = ctx.actions.declare_file("{}_pub_ids.h".format(ctx.label.name))
    pub_list_h = ctx.actions.declare_file("{}_publications.h".format(ctx.label.name))
    sub_ids_h = ctx.actions.declare_file("{}_sub_ids.h".format(ctx.label.name))
    sub_list_h = ctx.actions.declare_file("{}_subscriptions.h".format(ctx.label.name))
    dds_types_h = ctx.actions.declare_file("{}_dds_types.h".format(ctx.label.name))

    headers = {
        "publications": {
            "pub_ids": pub_ids_h.short_path,
            "pub_list": pub_list_h.short_path,
        },
        "subscriptions": {
            "sub_ids": sub_ids_h.short_path,
            "sub_list": sub_list_h.short_path,
        },
        "dds_types": dds_types_h.short_path,
    }

    args = ctx.actions.args()
    args.add("--yaml", ctx.file.yaml_config.path)
    args.add("--output_headers", headers)
    args.add("--output_dir", ctx.label.package)

    idl_paths = []
    for f in ctx.files.idls:
        idl_paths.append(f.path)

    args.add_all("--idl", idl_paths)
    ctx.actions.run(
        outputs = [pub_ids_h, pub_list_h, sub_ids_h, sub_list_h, dds_types_h],
        inputs = [ctx.file.yaml_config] + ctx.files.idls,
        executable = ctx.executable._generator,
        #workaround: there's some nasty toolchain misconfiguration
        #although, due to the nature of the script, it's ok 🐽
        use_default_shell_env = True,
        arguments = [args],
        mnemonic = "DDSPortsGen",
        progress_message = "Generating DDS Ports",
    )

    return [
        DefaultInfo(files = depset([pub_ids_h, pub_list_h, sub_ids_h, sub_list_h, dds_types_h])),
        CcInfo(
            compilation_context = cc_common.create_compilation_context(
                headers = depset([pub_ids_h, pub_list_h, sub_ids_h, sub_list_h, dds_types_h]),
                includes = depset([pub_ids_h.dirname]),
            ),
        ),
    ]

dds_ports = rule(
    implementation = _dds_ports_impl,
    attrs = {
        "yaml_config": attr.label(allow_single_file = [".yaml", ".yml"]),
        "idls": attr.label_list(allow_files = [".idl"]),
        "_generator": attr.label(
            default = Label("//core/communication/generators:dds_ports_generator"),
            executable = True,
            cfg = "exec",
        ),
    },
)
