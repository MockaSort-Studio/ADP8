"""This makes the linter happy"""

load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")

def pack_cc_library(ctx, srcs, hdrs, deps, includes = [], linkstatic = True):
    """Compiles C++ source files and produces standard Bazel providers.

    Args:
        ctx: The rule context (provides access to actions, label, and toolchain).
        srcs: List of File objects for the source (.cxx, .cpp, .c) files.
        hdrs: List of File objects for the public header (.hpp, .h, .ipp) files.
        deps: List of targets (from ctx.attr) that provide CcInfo.
        includes: List of strings representing include paths to add to the compilation.
        linkstatic: Boolean. If True, disallow dynamic linking for this target.

    Returns:
        A list containing DefaultInfo and CcInfo providers.
    """
    cc_toolchain = ctx.attr._cc_toolchain[cc_common.CcToolchainInfo]
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    deps_cc_info = [dep[CcInfo] for dep in deps]

    compilation_context, compilation_outputs = cc_common.compile(
        name = ctx.label.name,
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        srcs = srcs,
        public_hdrs = hdrs,
        includes = includes,
        compilation_contexts = [dep.compilation_context for dep in deps_cc_info],
    )

    linking_context, linking_outputs = cc_common.create_linking_context_from_compilation_outputs(
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        name = ctx.label.name,
        compilation_outputs = compilation_outputs,
        linking_contexts = [dep.linking_context for dep in deps_cc_info],
        disallow_dynamic_library = linkstatic,
    )

    output_libs = []
    if linking_outputs.library_to_link:
        lib = linking_outputs.library_to_link
        if lib.static_library:
            output_libs.append(lib.static_library)
        if lib.dynamic_library:
            output_libs.append(lib.dynamic_library)

    return [
        DefaultInfo(files = depset(srcs + hdrs + output_libs)),
        CcInfo(
            compilation_context = compilation_context,
            linking_context = linking_context,
        ),
    ]
