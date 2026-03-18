"""
FastDDS Gen rules.

This module provides codegen of FastDDS types from IDL definitions.

"""

load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")
load("//core/generators:cc_utils.bzl", "pack_cc_library")

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

    java_runtime = ctx.toolchains["@bazel_tools//tools/jdk:runtime_toolchain_type"].java_runtime

    java_args = ctx.actions.args()
    java_args.add("-jar")
    java_args.add(ctx.file._jar)

    ctx.actions.run(
        outputs = output_files,
        inputs = depset(idls + [ctx.file._jar], transitive = [java_runtime.files]),
        executable = java_runtime.java_executable_exec_path,
        arguments = [java_args, args],
        mnemonic = "FastDDSGen",
    )
    srcs = [f for f in output_files if f.extension == "cxx"]
    hdrs = [f for f in output_files if f.extension in ["hpp", "ipp"]]

    includes = [output_files[0].dirname] if output_files else []

    return pack_cc_library(
        ctx = ctx,
        srcs = srcs,
        hdrs = hdrs,
        deps = ctx.attr._deps,
        includes = includes,
        linkstatic = ctx.attr.linkstatic,
    )

cc_fastdds_types = rule(
    implementation = _cc_fastdds_types_impl,
    attrs = {
        "idl_srcs": attr.label_list(allow_files = True),
        "_deps": attr.label_list(providers = [CcInfo], default = ["@fastdds"]),
        "linkstatic": attr.bool(default = True),
        "_jar": attr.label(
            default = Label("@fastddsgen//:fastddsgen_jar"),
            allow_single_file = True,
        ),
        "_cc_toolchain": attr.label(default = Label("@bazel_tools//tools/cpp:current_cc_toolchain")),
    },
    fragments = ["cpp"],
    toolchains = [
        "@bazel_tools//tools/cpp:toolchain_type",
        "@bazel_tools//tools/jdk:runtime_toolchain_type",
    ],
)
