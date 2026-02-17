"""
macros to import tools easily in modules
"""

load("@aspect_bazel_lib//lib:expand_template.bzl", "expand_template_rule")
load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")
load("@rules_shell//shell:sh_binary.bzl", "sh_binary")

def create_aliases(name = "create_aliases"):
    native.alias(
        name = "format",
        actual = "//tools:format",
    )

def create_compile_commands(name = "compile_commands", targets = {}):
    refresh_compile_commands(
        name = name,
        # Specify the targets of interest.
        # For example, specify a dict of targets and any flags required to build.
        targets = targets,

        # No need to add flags already in .bazelrc. They're automatically picked up.
        # If you don't need flags, a list of targets is also okay, as is a single target string.
        # Wildcard patterns, like //... for everything, *are* allowed here, just like a build.
        # As are additional targets (+) and subtractions (-), like in bazel query https://docs.bazel.build/versions/main/query.html#expressions
        # And if you're working on a header-only library, specify a test or binary target that compiles it.
    )
    expand_template_rule(
        template = "//tools:fixup_compile_commands_transient.sh.in",
        is_executable = True,
        name = "fix_compile_commands_script",
        out = "fix_compile_commands.sh",
    )

    # this rules replaces the current directory (fastbuild, debug ecc...) with the bazel-bin symlink
    sh_binary(
        name = "{}_fix".format(name),
        srcs = [":fix_compile_commands_script"],
        data = [":{}".format(name), "compile_commands.json"],
    )
