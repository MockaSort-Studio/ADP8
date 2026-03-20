"""
Hermetic LLVM toolchain — repository rule + module extension.

Layout of each created repository:
  bin/      — wrapper scripts (one per tool, same names as the originals)
  bin-real/ — actual ELF binaries, invoked by the wrappers
  lib/      — bundled host runtime (glibc 2.36, libstdc++ 6.0.30, …)
  sysroot/  — Chromium Debian sysroot (TARGET: compiled code links against this)

Executor-agnostic: clang runs on any Linux regardless of system glibc.
Adding a platform: one entry in platforms.bzl, zero logic changes here.

Why wrappers and not patchelf?
  PT_INTERP (the ELF interpreter path) must be absolute.  The Bazel cache path
  is machine-specific, so we cannot embed it at fetch time.  Wrappers compute
  the absolute path at runtime via $(cd … && pwd) and invoke the bundled
  ld-linux with --library-path, bypassing the system interpreter entirely.

  $(cd … && pwd) is intentional — NOT readlink -f.  readlink -f resolves
  execroot symlinks to the real cache path, which breaks clang's resource-dir
  computation and triggers "absolute path inclusion" errors with -no-canonical-prefixes.

proudly AI-generated, human-reviewed
"""

load(":platforms.bzl", "LLVM_PLATFORMS")

# ── wrapper templates ─────────────────────────────────────────────────────────

# Generic wrapper for ELF binaries.  $(basename "$0") gives the real name after
# the binary has been moved to bin-real/.
_WRAPPER = """\
#!/bin/bash
# ROOT = toolchain root (contains bin-real/ and lib/).
# On local builds, non-clang tools may be invoked via a forwarding symlink
# from the toolchains_llvm repo; on RBE (BuildBuddy) Bazel uploads symlink
# *contents*, so the forwarding entry is a regular file — readlink returns
# nothing and any symlink-following strategy fails.
# Solution: when lib/ is not reachable from ROOT, search sibling repos under
# external/ for the one that owns lib/{ld_interp}.  Only our toolchain repo
# puts that file there, so the glob matches exactly one directory.
# Clang is unaffected: cc_wrapper.sh calls it with the direct repo path, so
# lib/ IS reachable and the search is never triggered — the logical execroot
# path (needed for clang's resource-dir) is preserved via cd+pwd (not readlink).
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
# Same ROOT-search logic; target binary name is hardcoded at fetch time.
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
filegroup(name = "clang-tidy",       srcs = ["bin/clang-tidy",      ":_bin_real", ":host_runtime_libs"])
filegroup(name = "clang-format",     srcs = ["bin/clang-format",    ":_bin_real", ":host_runtime_libs"])
filegroup(name = "clangd",           srcs = ["bin/clangd",          ":_bin_real", ":host_runtime_libs"])
filegroup(name = "git-clang-format", srcs = ["bin/git-clang-format",             ":host_runtime_libs"])
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
data=$(ar t {deb} | grep '^data\\.tar\\.')
case "$data" in
    *.xz) ar p {deb} "$data" | tar xJf - -O '{inner}' > '{dest}' ;;
    *.gz) ar p {deb} "$data" | tar xzf - -O '{inner}' > '{dest}' ;;
    *)    echo "unsupported deb data archive: $data" >&2; exit 1 ;;
esac
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
        rctx.file(elf, _WRAPPER.format(ld_interp = cfg.ld_interp), executable = True)

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
