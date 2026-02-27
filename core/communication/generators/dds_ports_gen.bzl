load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")

def _dds_ports_impl(ctx):
    pub_ids_h = ctx.actions.declare_file("{}_pub_ids.hpp".format(ctx.label.name))
    pub_list_h = ctx.actions.declare_file("{}_publications.hpp".format(ctx.label.name))
    sub_ids_h = ctx.actions.declare_file("{}_sub_ids.hpp".format(ctx.label.name))
    sub_list_h = ctx.actions.declare_file("{}_subscriptions.hpp".format(ctx.label.name))
    dds_types_h = ctx.actions.declare_file("{}_dds_types.hpp".format(ctx.label.name))

    headers = {
        "publications": {
            "ids": pub_ids_h.path,
            "specs": pub_list_h.path,
        },
        "subscriptions": {
            "ids": sub_ids_h.path,
            "specs": sub_list_h.path,
        },
        "dds_types": dds_types_h.path,
    }

    args = ctx.actions.args()
    args.add("--yaml", ctx.file.yaml_config.path)
    args.add("--outputs", json.encode(headers))

    idl_paths = []
    for f in ctx.files.idls:
        idl_paths.append(f.path)

    outputs = [pub_ids_h, pub_list_h, sub_ids_h, sub_list_h, dds_types_h]
    args.add_all("--idl", idl_paths)
    ctx.actions.run(
        outputs = outputs,
        inputs = [ctx.file.yaml_config] + ctx.files.idls + ctx.files._templates,
        executable = ctx.executable._generator,
        #workaround: there's some nasty toolchain misconfiguration
        #although, due to the nature of the script, it's ok 🐽
        use_default_shell_env = True,
        arguments = [args],
        mnemonic = "DDSPortsGen",
        progress_message = "Generating DDS Ports",
    )

    compilation_context = cc_common.create_compilation_context(
        headers = depset(outputs),
        includes = depset([f.dirname for f in outputs]),
    )

    return [
        DefaultInfo(files = depset(outputs)),
        CcInfo(
            compilation_context = cc_common.merge_compilation_contexts(
                compilation_contexts = [compilation_context] + [
                    dep[CcInfo].compilation_context
                    for dep in ctx.attr.deps
                    if CcInfo in dep
                ],
            ),
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
        "deps": attr.label_list(providers = [CcInfo]),
    },
)
