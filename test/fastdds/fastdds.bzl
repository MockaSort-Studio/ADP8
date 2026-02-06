"""
FastDDS Gen rules.

This module provides codegen of FastDDS types from IDL definitions.

"""

def _cc_fastdds_types_impl(ctx):
    idls = ctx.files.idl_srcs
    idls_paths = [idl.path for idl in idls]

    output_files = []

    suffixes = [
        #idl_name.hpp
        ".hpp",
        #idl_nameTypeObjectSupport.cxx/.hpp
        "TypeObjectSupport.cxx",
        "TypeObjectSupport.hpp",
        #idl_namePubSubTypes.cxx/.hpp
        "PubSubTypes.cxx",
        "PubSubTypes.hpp",
        #idl_nameCdrAux.ipp/.hpp
        "CdrAux.ipp",
        "CdrAux.hpp",
    ]
    for idl in idls:
        base_name = idl.basename.rsplit(".", 1)[0]
        for suffix in suffixes:
            output_files.append(ctx.actions.declare_file("{base_name}{suffix}".format(base_name = base_name, suffix = suffix)))

    # Fast DDS Gen args
    args = ctx.actions.args()

    # this makes sure all output files go in the folder specified by -d
    args.add("-flat-output-dir")

    # disable preprocessor, it does not work properly in the bazel sandbox + clang
    # it's disabled since for now idl files do not declare any preprocessor directives
    args.add("-ppDisable")

    #destination directory
    args.add("-d")
    args.add(output_files[0].dirname)

    #input IDL files
    args.add_all(idls_paths)

    ctx.actions.run(
        outputs = output_files,
        inputs = idls,
        executable = ctx.executable._generator,
        arguments = [args],
        mnemonic = "FastDDSGen",
    )
    srcs = [f for f in output_files if f.extension == "cxx"]
    hdrs = [f for f in output_files if f.extension in ["hpp", "ipp"]]

    deps_cc_info = [dep[CcInfo] for dep in ctx.attr.deps]

    cc_toolchain = ctx.attr._cc_toolchain[cc_common.CcToolchainInfo]
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
        srcs = srcs,
        public_hdrs = hdrs,
        includes = [output_files[0].dirname],
        compilation_contexts = [dep.compilation_context for dep in deps_cc_info],
    )

    linking_context, linking_outputs = cc_common.create_linking_context_from_compilation_outputs(
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        name = ctx.label.name,
        compilation_outputs = compilation_outputs,
        linking_contexts = [dep.linking_context for dep in deps_cc_info],
        disallow_dynamic_library = ctx.attr.linkstatic,
    )

    all_outputs = [f for f in output_files]
    if linking_outputs.library_to_link:
        lib = linking_outputs.library_to_link
        if lib.static_library:
            all_outputs.append(lib.static_library)
        if lib.dynamic_library:
            all_outputs.append(lib.dynamic_library)

    return [
        DefaultInfo(files = depset(all_outputs)),
        CcInfo(
            compilation_context = compilation_context,
            linking_context = linking_context,
        ),
    ]

cc_fastdds_types = rule(
    implementation = _cc_fastdds_types_impl,
    attrs = {
        "idl_srcs": attr.label_list(allow_files = True),
        "deps": attr.label_list(providers = [CcInfo], default = ["@fastdds"]),
        "linkstatic": attr.bool(default = True),
        "_generator": attr.label(
            executable = True,
            cfg = "exec",
            default = Label("//tools:fastddsgen"),
        ),
        "_cc_toolchain": attr.label(default = Label("@bazel_tools//tools/cpp:current_cc_toolchain")),
    },
    fragments = ["cpp"],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
)
