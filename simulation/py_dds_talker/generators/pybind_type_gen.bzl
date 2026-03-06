"""Bazel rule: generates a pybind11 .cpp type binding from an IDL file."""

def _pybind_type_impl(ctx):
    output = ctx.actions.declare_file("{}.cpp".format(ctx.label.name))

    args = ctx.actions.args()
    args.add("--modality", "type")
    args.add_all("--idls", ctx.files.idls)
    args.add("--output", output.path)

    ctx.actions.run(
        outputs = [output],
        inputs = depset(ctx.files.idls + ctx.files._templates),
        executable = ctx.executable._generator,
        arguments = [args],
        mnemonic = "PyBindTypeGen",
        progress_message = "Generating pybind11 type binding for {}".format(ctx.label.name),
    )

    return [DefaultInfo(files = depset([output]))]

pybind_type_gen = rule(
    implementation = _pybind_type_impl,
    attrs = {
        "idls": attr.label_list(allow_files = [".idl"]),
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
