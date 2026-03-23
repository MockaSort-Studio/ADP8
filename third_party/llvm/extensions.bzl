"""
Hermetic LLVM toolchain — repository rule + module extension.

Layout of each created repository:
  bin/      — wrapper scripts (one per tool, same names as the originals)
  bin-real/ — actual ELF binaries, invoked by the wrappers
  lib/      — bundled host runtime (glibc 2.36, libstdc++ 6.0.30, …)
  sysroot/  — Chromium Debian sysroot (TARGET: compiled code links against this)

Executor-agnostic: clang runs on any Linux regardless of system glibc.
Adding a platform: one entry in platforms.bzl, zero logic changes here.

Two wrapper flavors:

  _WRAPPER (compilation tools — clang, lld, llvm-ar, …)
    Uses $(cd … && pwd) (logical path, NOT readlink -f).
    readlink -f would resolve execroot symlinks to the real Bazel cache path,
    breaking clang's resource-dir computation and causing "absolute path
    inclusion" errors under -no-canonical-prefixes on RBE.
    bin-real/ must be in the filegroup so sandbox/RBE actions can find the
    ELFs via the logical runfiles path.

  _WRAPPER_STANDALONE (standalone tools — clang-format, clangd, clang-tidy)
    Uses readlink -f (physical path).
    These tools do not derive resource-dir or include paths from argv[0], so
    physical-path resolution is safe.  The physical path points directly into
    the Bazel external cache, so bin-real/ does NOT need to be in the
    filegroup — format_multirun (and other callers) see only bin/<tool> and
    cannot accidentally pick up the raw ELF, which would pair the system
    ld-linux with our bundled libc and cause stack smashing.

proudly AI-generated, human-reviewed
"""

load(":platforms.bzl", "LLVM_PLATFORMS")

# ── wrapper templates ─────────────────────────────────────────────────────────

# Generic wrapper for compilation-toolchain ELF binaries.
# ROOT is the logical path (cd+pwd, NOT readlink): this preserves the execroot
# symlink chain that clang uses to locate its own resource directory.
_WRAPPER = """\
#!/bin/bash
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [ ! -f "$ROOT/lib/{ld_interp}" ]; then
    _ext="$(cd "$ROOT/.." && pwd)"
    for _d in "$_ext"/*/lib/{ld_interp}; do
        [ -f "$_d" ] || continue
        ROOT="$(cd "$(dirname "$_d")/.." && pwd)"
        break
    done
fi
exec "$ROOT/lib/{ld_interp}" \\
     --library-path "$ROOT/lib" \\
     "$ROOT/bin-real/$(basename "$0")" \\
     "$@"
"""

# Wrapper for symlinks (e.g. clang → clang-19, ld.lld → lld).
# Same logical-path ROOT search; target binary name is hardcoded at fetch time.
# --argv0 "$0" passes the wrapper's own full path as argv[0] to the ELF.
# This matters for tools that dispatch on argv[0] basename (e.g. lld selects
# ld.lld / lld-link / wasm-ld mode from it).  Using the full path (not bare
# name) avoids PATH lookup under -no-canonical-prefixes, which would find a
# system binary on the executor instead of our hermetic one.
_WRAPPER_SYMLINK = """\
#!/bin/bash
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [ ! -f "$ROOT/lib/{ld_interp}" ]; then
    _ext="$(cd "$ROOT/.." && pwd)"
    for _d in "$_ext"/*/lib/{ld_interp}; do
        [ -f "$_d" ] || continue
        ROOT="$(cd "$(dirname "$_d")/.." && pwd)"
        break
    done
fi
exec "$ROOT/lib/{ld_interp}" \\
     --argv0 "$0" \\
     --library-path "$ROOT/lib" \\
     "$ROOT/bin-real/{target}" \\
     "$@"
"""

# Wrapper for standalone tools (clang-format, clangd, clang-tidy).
# Uses readlink -f to resolve the physical Bazel cache path regardless of
# how the script is invoked ($0 may be a runfiles symlink, a sandbox path,
# or a plain path on an RBE executor — readlink -f handles all three).
# Physical-path resolution is safe for these tools: they do not derive
# include paths or resource directories from argv[0].
# Because ROOT is the physical cache path, bin-real/<tool> is always
# reachable without a runfiles symlink, so the filegroups for these tools
# intentionally omit :_bin_real.
_WRAPPER_STANDALONE = """\
#!/bin/bash
REAL="$(readlink -f "$0")"
ROOT="$(cd "$(dirname "$REAL")/.." && pwd)"
exec "$ROOT/lib/{ld_interp}" \\
     --library-path "$ROOT/lib" \\
     "$ROOT/bin-real/{name}" \\
     "$@"
"""

# Standalone tools that should receive the readlink-f wrapper.
_STANDALONE_ELF_TOOLS = {
    "clang-format": True,
    "clangd": True,
    "clang-tidy": True,
}

# ── BUILD templates ───────────────────────────────────────────────────────────

_LLVM_BUILD = """\
package(default_visibility = ["//visibility:public"])

exports_files(glob(
    ["bin/*", "bin-real/**", "lib/**", "include/**", "share/clang/*"],
    allow_empty = True,
))

# All bundled host runtime libs — glob picks up everything copied from debs
# and sysroot, so adding a new lib in platforms.bzl requires no BUILD change.
filegroup(
    name = "host_runtime_libs",
    srcs = glob(["lib/lib*.so*", "lib/ld*.so*"], allow_empty = True),
)

filegroup(name = "_bin_real", srcs = glob(["bin-real/**"]))

filegroup(
    name = "clang",
    srcs = ["bin/clang", "bin/clang++", "bin/clang-cpp", "bin/clang-19",
            ":_bin_real", ":host_runtime_libs"],
)

filegroup(
    name = "ld",
    srcs = ["bin/ld.lld", "bin/ld64.lld", ":_bin_real", ":host_runtime_libs"]
          + glob(["bin/wasm-ld"], allow_empty = True),
)

filegroup(name = "include",             srcs = glob(["include/**/c++/**", "lib/clang/*/include/**"]))
filegroup(name = "all_includes",        srcs = glob(["include/**"], allow_empty = True))
filegroup(name = "cxx_builtin_include", srcs = ["include/c++", "lib/clang/19/include"])
filegroup(name = "extra_config_site",   srcs = glob(["include/*/c++/v1/__config_site"], allow_empty = True))
filegroup(name = "bin",                 srcs = glob(["bin/**"]))

filegroup(
    name = "lib",
    srcs = ["lib/clang/19/lib", ":host_runtime_libs"]
          + glob(["lib/**/libc++*.a", "lib/**/libunwind.a"]),
)

filegroup(
    name = "lib_legacy",
    srcs = glob(["lib/clang/19/lib/**", "lib/**/libc++*.a", "lib/**/libunwind.a"]),
)

filegroup(name = "ar",               srcs = ["bin/llvm-ar",        ":_bin_real", ":host_runtime_libs"])
filegroup(name = "as",               srcs = ["bin/clang", "bin/llvm-as", ":_bin_real", ":host_runtime_libs"])
filegroup(name = "nm",               srcs = ["bin/llvm-nm",         ":_bin_real", ":host_runtime_libs"])
filegroup(name = "objcopy",          srcs = ["bin/llvm-objcopy",    ":_bin_real", ":host_runtime_libs"])
filegroup(name = "objdump",          srcs = ["bin/llvm-objdump",    ":_bin_real", ":host_runtime_libs"])
filegroup(name = "profdata",         srcs = ["bin/llvm-profdata",   ":_bin_real", ":host_runtime_libs"])
filegroup(name = "dwp",              srcs = ["bin/llvm-dwp",        ":_bin_real", ":host_runtime_libs"])
filegroup(name = "ranlib",           srcs = ["bin/llvm-ranlib",     ":_bin_real", ":host_runtime_libs"])
filegroup(name = "readelf",          srcs = ["bin/llvm-readelf",    ":_bin_real", ":host_runtime_libs"])
filegroup(name = "strip",            srcs = ["bin/llvm-strip",      ":_bin_real", ":host_runtime_libs"])
filegroup(name = "symbolizer",       srcs = ["bin/llvm-symbolizer", ":_bin_real", ":host_runtime_libs"])

# Standalone tools used outside the compilation toolchain (format_multirun,
# VSCode extensions, IDE integrations).  No :_bin_real: the _WRAPPER_STANDALONE
# scripts use readlink -f to find the physical Bazel cache path directly, so
# the ELF does not need to be present in the runfiles/sandbox file tree.
# Omitting :_bin_real also prevents format_multirun from picking bin-real/<tool>
# (the raw ELF) before bin/<tool> (the wrapper), which would pair the system
# ld-linux with our bundled libc — a mismatched glibc build — and crash.
filegroup(name = "clang-tidy",       srcs = ["bin/clang-tidy",      ":host_runtime_libs"])
filegroup(name = "clang-format",     srcs = ["bin/clang-format",    ":host_runtime_libs"])
filegroup(name = "clangd",           srcs = ["bin/clangd",          ":host_runtime_libs"])
filegroup(name = "git-clang-format", srcs = ["bin/git-clang-format",":host_runtime_libs"])
filegroup(name = "libclang",         srcs = glob(["lib/libclang.so", "lib/libclang.dylib"], allow_empty = True))

# Anchor label for llvm.toolchain_root() in MODULE.bazel.
filegroup(name = "dummy", srcs = [])
"""

_SYSROOT_BUILD = """\
package(default_visibility = ["//visibility:public"])

filegroup(
    name = "sysroot",
    srcs = glob(["**/*"]),
)
"""

# ── repository rule implementation ───────────────────────────────────────────

def _llvm_repo_impl(rctx):
    cfg = LLVM_PLATFORMS[rctx.attr.platform]

    # ── 1. LLVM release tarball ───────────────────────────────────────────────
    rctx.download_and_extract(
        url = cfg.llvm_url,
        sha256 = cfg.llvm_sha256,
        stripPrefix = cfg.llvm_strip_prefix,
    )

    # ── 2. Target sysroot ─────────────────────────────────────────────────────
    rctx.download_and_extract(
        url = cfg.sysroot_url,
        sha256 = cfg.sysroot_sha256,
        output = "sysroot",
    )

    # ── 3. Extra libs from sysroot (not covered by the debs) ──────────────────
    for (src_dir, glob_pattern, dest) in cfg.sysroot_libs:
        result = rctx.execute([
            "sh",
            "-c",
            "find sysroot/{} -name '{}' -not -type l".format(src_dir, glob_pattern),
        ])
        if result.return_code != 0 or not result.stdout.strip():
            fail("could not find {} in sysroot/{}".format(glob_pattern, src_dir))
        matches = [f for f in result.stdout.strip().split("\n") if f.strip()]
        if len(matches) > 1:
            fail("ambiguous sysroot lib {}: multiple matches: {}".format(glob_pattern, matches))
        rctx.execute(["cp", matches[0], dest])
        rctx.execute(["chmod", "+x", dest])

    # ── 4. Host runtime debs (glibc 2.36 — clang needs this to execute) ───────
    for deb in cfg.host_runtime_debs:
        rctx.download(url = deb.url, sha256 = deb.sha256, output = deb.name)
        for (inner, dest) in deb.extractions:
            result = rctx.execute(["sh", "-c", """\
dpkg-deb --fsys-tarfile {deb} | tar xf - -O '{inner}' > '{dest}'
""".format(deb = deb.name, inner = inner, dest = dest)])
            if result.return_code != 0:
                fail("extracting {} from {}: {}".format(inner, deb.name, result.stderr))
            rctx.execute(["chmod", "+x", dest])
        rctx.execute(["rm", deb.name])

    # ── 5. Wrap symlinks — resolve full chain, hardcode final target ──────────
    # Must happen before step 6 so the symlink targets still exist in bin/.
    # The chain walk handles multi-hop cases (clang++ → clang → clang-19).
    rctx.execute(["mkdir", "-p", "bin-real"])

    sym_script = """\
find bin -maxdepth 1 -type l | while IFS= read -r sym; do
    name=$(basename "$sym")
    target=$name
    while [ -L "bin/$target" ]; do
        next=$(readlink "bin/$target")
        target=$(basename "$next")
    done
    printf '%s %s\\n' "$sym" "$target"
done
"""
    result = rctx.execute(["sh", "-c", sym_script])
    for ln in [line.strip() for line in result.stdout.strip().split("\n") if line.strip()]:
        parts = ln.split(" ", 1)
        if len(parts) != 2:
            continue
        sym_path, target = parts[0], parts[1]
        name = sym_path.split("/")[-1]
        rctx.execute(["rm", sym_path])
        wrapper = _WRAPPER_SYMLINK.format(ld_interp = cfg.ld_interp, target = target)
        rctx.file(sym_path, wrapper, executable = True)

        # clang's driver resolves its tool-search directory from argv[0].
        # When invoked as bin-real/clang-19 it looks for ld.lld (and other
        # tools) in bin-real/, not bin/.  Mirror every symlink wrapper there
        # so the lookup succeeds and the bundled glibc wrapper is used.
        rctx.file("bin-real/" + name, wrapper, executable = True)

    # ── 6. Move ELF binaries → bin-real/, install generic wrappers ───────────
    # ELF detection via magic bytes — no dependency on `file`.
    result = rctx.execute(["sh", "-c", """\
find bin -maxdepth 1 -type f | while IFS= read -r f; do
    [ "$(od -A n -N 4 -t x1 "$f" | tr -d ' \\n')" = "7f454c46" ] && echo "$f"
done
"""])
    for elf in [f.strip() for f in result.stdout.strip().split("\n") if f.strip()]:
        name = elf.split("/")[-1]
        rctx.execute(["mv", elf, "bin-real/" + name])
        if name in _STANDALONE_ELF_TOOLS:
            wrapper = _WRAPPER_STANDALONE.format(ld_interp = cfg.ld_interp, name = name)
        else:
            wrapper = _WRAPPER.format(ld_interp = cfg.ld_interp)
        rctx.file(elf, wrapper, executable = True)

    rctx.file("BUILD.bazel", _LLVM_BUILD)
    rctx.file("sysroot/BUILD.bazel", _SYSROOT_BUILD)

_llvm_repo = repository_rule(
    implementation = _llvm_repo_impl,
    attrs = {"platform": attr.string(mandatory = True)},
)

# ── module extension ──────────────────────────────────────────────────────────

def _llvm_extension_impl(_ctx):
    for platform in LLVM_PLATFORMS:
        _llvm_repo(
            name = "llvm_toolchain_" + platform.replace("-", "_"),
            platform = platform,
        )

llvm_toolchain = module_extension(
    implementation = _llvm_extension_impl,
)
