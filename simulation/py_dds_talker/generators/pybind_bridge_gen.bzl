"""Bazel rule: generates a pybind11 .cpp PyDDSBridge binding from a ports YAML."""

def _pybind_bridge_impl(ctx):
    output = ctx.actions.declare_file("{}.cpp".format(ctx.label.name))

    args = ctx.actions.args()
    args.add("--modality", "bridge")
    args.add("--yaml", ctx.file.yaml_config.path)
    args.add_all("--idls", ctx.files.idls)
    args.add("--namespace", ctx.attr.namespace)
    args.add("--ports-name", ctx.attr.ports_name)
    args.add("--module-name", ctx.attr.module_name if ctx.attr.module_name else ctx.label.name)
    args.add("--output", output.path)

    ctx.actions.run(
        outputs = [output],
        inputs = depset(
            [ctx.file.yaml_config] + ctx.files.idls + ctx.files._templates,
        ),
        executable = ctx.executable._generator,
        arguments = [args],
        mnemonic = "PyBindBridgeGen",
        progress_message = "Generating pybind11 bridge binding for {}".format(ctx.label.name),
    )

    return [DefaultInfo(files = depset([output]))]

pybind_bridge_gen = rule(
    implementation = _pybind_bridge_impl,
    attrs = {
        "yaml_config": attr.label(allow_single_file = [".yaml", ".yml"]),
        "idls": attr.label_list(allow_files = [".idl"]),
        "namespace": attr.string(default = "gen"),
        "ports_name": attr.string(),
        "module_name": attr.string(default = ""),
        "_generator": attr.label(
            default = Label("//simulation/py_dds_talker/generators:pybind_generators"),
            executable = True,
            cfg = "exec",
        ),
        "_templates": attr.label(
            default = Label("//simulation/py_dds_talker/generators:jinja_templates"),
            allow_files = [".jinja"],
        ),
    },
)
