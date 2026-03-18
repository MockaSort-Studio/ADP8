# Hermetic LLVM Toolchain

proudly AI-generated, human-reviewed

Hermetic LLVM19 toolchain with sysroot made to ease deployment and enable headache free RBE.

Built for [llvm_toolchain](https://github.com/bazel-contrib/toolchains_llvm).

It implements its own sysroot starting from [Chromium Debian Bookwork Sysroot](https://github.com/chromium/chromium/blob/main/docs/linux/sysroot.md) and adding .deb packages to cover system cland deps

---

## Repo layout (per platform)

Each platform creates one Bazel repository with this structure:

```
llvm_toolchain_linux_x86_64/
├── bin/          # bash wrappers (one per tool, same names as originals)
├── bin-real/     # actual ELF binaries + copies of symlink wrappers
├── lib/          # bundled host runtime (glibc 2.36, libstdc++ 6.0.30, lld deps, …)
└── sysroot/      # Chromium Debian Bullseye sysroot (TARGET ABI)
```

`bin/` contains no ELF files — only bash scripts. The real binaries live in `bin-real/`.

---

## Why wrappers

ELF binaries have a hardcoded interpreter path (`PT_INTERP`, e.g. `/lib64/ld-linux-x86-64.so.2`).
On a system with a different or missing glibc that path doesn't exist and the binary won't run.
The fix is to invoke the binary explicitly via our bundled `ld-linux.so`, but that path must be
absolute — and the Bazel cache path is machine-specific, so it can't be embedded at fetch time.

Each wrapper resolves its own location at runtime and calls the ELF through the bundled interpreter:

```bash
exec "$ROOT/lib/ld-linux-x86-64.so.2" \
     --library-path "$ROOT/lib" \
     "$ROOT/bin-real/clang-19" \
     "$@"
```

`$(cd … && pwd)` instead of `readlink -f`: `readlink -f` resolves execroot symlinks to the physical
Bazel cache path. clang derives its resource directory from `argv[0]`; a physical path breaks that
computation and causes "absolute path inclusion" errors with `-no-canonical-prefixes`.

## Two wrapper variants

There are two wrapper templates because `argv[0]` needs to be set differently depending on the tool.

| Variant | File | `argv[0]` passed to ELF | When used |
|---------|------|------------------------|-----------|
| `_WRAPPER` | `bin/clang-19`, `bin/llvm-ar`, … | ld-linux default (full ELF path, has slashes → no PATH lookup) | ELF binaries moved to `bin-real/` |
| `_WRAPPER_SYMLINK` | `bin/clang`, `bin/ld.lld`, … | `"$0"` (full wrapper path, correct basename) | Former symlinks; `--argv0 "$0"` needed so lld dispatches on correct mode |

lld is a multi-mode driver that picks its mode from `argv[0]` basename: `ld.lld`=ELF linker,
`lld-link`=COFF, `wasm-ld`=WebAssembly, `lld`=error. Without `--argv0 "$0"`, ld-linux would pass
the real ELF path (`bin-real/lld`) whose basename is `lld` — the generic driver that just prints
an error and exits.

## RBE forwarding-repo fallback

On local builds, `toolchains_llvm` may invoke tools through a forwarding repo (a different Bazel
external repo that symlinks back to the toolchain repo). On RBE (BuildBuddy), Bazel uploads symlink
*contents* as regular files — there is no symlink to follow — so the wrapper ends up in the wrong
directory and can't find `lib/`. Both wrappers contain a fallback: if `lib/ld-linux-x86-64.so.2`
isn't reachable from the current ROOT, scan all sibling repos under `external/` for one that has it.
Only the hermetic toolchain repo puts that file there, so the glob matches exactly one.

```bash
if [ ! -f "$ROOT/lib/{ld_interp}" ]; then
    _ext="$(cd "$ROOT/.." && pwd)"
    for _d in "$_ext"/*/lib/{ld_interp}; do
        [ -f "$_d" ] || continue
        ROOT="$(cd "$(dirname "$_d")/.." && pwd)"
        break
    done
fi
```

## `bin-real/` mirroring

clang's driver derives its tool-search directory from `argv[0]`. When Bazel invokes clang as
`bin-real/clang-19` (which happens after the wrapper layer for some code paths), clang looks for
`ld.lld` and other tools in `bin-real/`, not `bin/`. To cover this, every symlink wrapper written
to `bin/` is also written to `bin-real/` during fetch (step 5 in `_llvm_repo_impl`).

## /proc/self/cwd include hack

BuildBuddy mounts the RBE sandbox at `/proc/self/cwd`, which is a real directory on Linux (via
procfs), not just the usual symlink. `cc_wrapper.sh` computes a relative toolchain path prefix;
clang resolves that against the CWD and writes the full `/proc/self/cwd/external/…` path into `.d`
(dependency) files. Bazel then validates those paths client-side against the local execroot, which
doesn't have `/proc/self/cwd/` — causing "absolute path inclusion" errors on every header.

Fix: add `/proc/self/cwd` to `cxx_builtin_include_directories` in `MODULE.bazel`. This tells Bazel
to accept any `.d` path under that prefix as a known builtin include directory. The entry passes
`toolchains_llvm`'s `_is_hermetic_or_exists` filter because `/proc/self/cwd` genuinely exists on
Linux; it is filtered out harmlessly on macOS.

```python
cxx_builtin_include_directories = {
    "linux-x86_64": ["/proc/self/cwd"],
}
```

---

## Host runtime bundled in `lib/`

The sysroot defines what ABI compiled binaries target (Bullseye, glibc ≥ 2.31). That's separate
from what the LLVM *tools themselves* need to run. LLVM 19 requires glibc 2.34 and libstdc++ 6.0.30
at minimum — not guaranteed on arbitrary executors. All tool dependencies are pulled from Debian
Bookworm debs and bundled so the tools are fully self-contained.

| Library | Version | Needed by |
|---------|---------|-----------|
| `libc.so.6`, `libm.so.6`, `ld-linux-x86-64.so.2` | glibc 2.36 (Bookworm) | all LLVM tools |
| `libstdc++.so.6` | 12.2.0 (LLVM 19 needs ≤ 6.0.30) | all LLVM tools |
| `libgcc_s.so.1` | 12.2.0 | all LLVM tools |
| `libxml2.so.2` | 2.9.14 | lld (MachO TBD parsing) |
| `libicuuc.so.72`, `libicudata.so.72` | ICU 72.1 (Bookworm) | libxml2 — system ICU 74 needs GLIBC_2.38, too new |
| `liblzma.so.5` | 5.4.1 | libxml2 |
| `libzstd.so.1`, `libz.so.1` | from sysroot | lld |

---

## Known constraints

- `_LLVM_BUILD` hardcodes `clang/19` in filegroup srcs — must update manually on LLVM version bump.
- `/proc/self/cwd` trick is BuildBuddy-specific; other RBE providers may need different treatment.
- Wrapper fallback silently proceeds with the wrong ROOT if no sibling repo has `lib/{ld_interp}` —
  the resulting error is a confusing "not found" from ld-linux, not an explicit message.
- Deb extraction only handles `.xz` and `.gz` archives. `.zst` (used by some newer Debian releases)
  will error explicitly — add `zstd -cd` handling if you ever need a Trixie+ deb.

---

## How to add a bundled host library

A tool added to the build fails at runtime with `error while loading shared libraries: libfoo.so.1`.
Find the Bookworm deb, add an entry to `host_runtime_debs` in `platforms.bzl`. Nothing else changes.

```python
struct(
    name = "libfoo.deb",
    url  = "https://ftp.debian.org/debian/pool/main/f/foo/libfoo1_1.2.3_amd64.deb",
    sha256 = "...",
    extractions = [
        ("./usr/lib/x86_64-linux-gnu/libfoo.so.1.2.3", "lib/libfoo.so.1"),
    ],
),
```

- Use Bookworm debs (glibc 2.36). Trixie debs may require a newer glibc than what's bundled.
- Verify the actual path inside the archive before committing:
  `ar p foo.deb data.tar.xz | tar tJf - | grep libfoo`
- Pre-usrmerge debs: `./lib/x86_64-linux-gnu/`. Post-usrmerge: `./usr/lib/x86_64-linux-gnu/`. Check.
- `host_runtime_libs` filegroup uses `glob(["lib/lib*.so*", "lib/ld*.so*"])` — no BUILD change needed.

## How to add a sysroot lib (not in any deb)

Some libraries exist only in the sysroot and have no matching Bookworm deb. Add to `sysroot_libs`:

```python
sysroot_libs = [
    ("usr/lib/x86_64-linux-gnu", "libfoo.so.1.*", "lib/libfoo.so.1"),
    ...
],
```

Fields: `(src_dir_in_sysroot, glob_pattern, dest_in_lib)`. First match wins; no warning if multiple.

## How to upgrade LLVM

1. `platforms.bzl`: update `llvm_url`, `llvm_sha256`, `llvm_strip_prefix`.
2. `extensions.bzl` `_LLVM_BUILD` template: replace three occurrences of `clang/19` with `clang/NN`.
   Search for `/19` in the template string.
3. `MODULE.bazel`: update `llvm_versions`.
4. Check `libstdc++.so` extraction version — it's versioned (`libstdc++.so.6.0.30`), must match the deb.

## How to add a platform (e.g. linux-aarch64)

`extensions.bzl` iterates `LLVM_PLATFORMS` — adding a platform is purely a data change.

1. Uncomment and fill the `linux-aarch64` stub in `platforms.bzl`:
   - `llvm_sha256`: from the GitHub release page for the ARM64 tarball.
   - `sysroot_url/sha256`: from Chromium's sysroot builder or Debian multiarch.
   - `ld_interp = "ld-linux-aarch64.so.1"`.
   - `host_runtime_debs`: same structure, use `_arm64.deb` package variants.
2. `MODULE.bazel`: add `linux-aarch64` to `cxx_builtin_include_directories`, `llvm.toolchain_root`,
   and `llvm.sysroot`.
3. `.bazelrc`: add `--platforms=@toolchains_llvm//platforms:linux-aarch64` where needed.
4. `extensions.bzl`: no changes.
