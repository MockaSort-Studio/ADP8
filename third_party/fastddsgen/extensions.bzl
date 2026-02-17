def _local_tool_repo_impl(repository_ctx):
    repository_ctx.extract(
        archive = repository_ctx.path(repository_ctx.attr.src),
    )

    repository_ctx.symlink(
        repository_ctx.path(repository_ctx.attr.build_file),
        "BUILD",
    )

_fastddsgen_archive = repository_rule(
    implementation = _local_tool_repo_impl,
    attrs = {
        "src": attr.label(mandatory = True, allow_single_file = [".tar.xz"]),
        "build_file": attr.label(mandatory = True, allow_single_file = True),
    },
)

def _fastddsgen_impl(ctx):
    _fastddsgen_archive(
        name = "fastddsgen",
        src = "//:third_party/fastddsgen/fastddsgen.tar.xz",
        build_file = "//:third_party/fastddsgen/fastddsgen.BUILD",
    )

fastddsgen = module_extension(
    implementation = _fastddsgen_impl,
)
