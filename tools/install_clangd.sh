#!/usr/bin/bash
# proudly AI-generated, human-reviewed
#
# Installs hermetic clangd and clang-format from the registered LLVM 19
# toolchain into .bin/ at the workspace root.
#
# Run via: bazel run //:install_clangd
# Intended as devcontainer postCreateCommand.
#
# A symlink to the wrapper script would NOT work here: the wrapper locates
# lib/ via dirname($0), which when invoked from .bin/ resolves to the
# workspace root — no lib/ there, and the fallback glob doesn't reach the
# Bazel cache.  Instead we write a thin launcher script that hardcodes the
# toolchain root at install time.  Re-run after bazel clean --expunge.

set -euo pipefail

# --- begin runfiles.bash initialization ---
# shellcheck source=/dev/null
source "${RUNFILES_DIR:-"${BASH_SOURCE[0]}.runfiles"}/bazel_tools/tools/bash/runfiles/runfiles.bash"
# --- end runfiles.bash initialization ---

WORKSPACE_ROOT="${BUILD_WORKSPACE_DIRECTORY}"
mkdir -p "${WORKSPACE_ROOT}/.bin"

# Resolve each wrapper to its canonical path in the Bazel external cache,
# then compute the toolchain root (one level above bin/).
_resolve_root() {
	realpath "$(rlocation "$1")" | xargs dirname | xargs -I{} sh -c 'cd "{}" && cd .. && pwd'
}

CLANGD_ROOT="$(_resolve_root llvm_toolchain_linux_x86_64/bin/clangd)"
CLANGFORMAT_ROOT="$(_resolve_root llvm_toolchain_linux_x86_64/bin/clang-format)"

# Write launcher scripts with hardcoded paths.  Using the bundled linker
# directly means the scripts are self-contained and need no PATH tricks.
cat >"${WORKSPACE_ROOT}/.bin/clangd" <<EOF
#!/bin/bash
exec "${CLANGD_ROOT}/lib/ld-linux-x86-64.so.2" \\
     --library-path "${CLANGD_ROOT}/lib" \\
     "${CLANGD_ROOT}/bin-real/clangd" \\
     "\$@"
EOF
chmod +x "${WORKSPACE_ROOT}/.bin/clangd"

cat >"${WORKSPACE_ROOT}/.bin/clang-format" <<EOF
#!/bin/bash
exec "${CLANGFORMAT_ROOT}/lib/ld-linux-x86-64.so.2" \\
     --library-path "${CLANGFORMAT_ROOT}/lib" \\
     "${CLANGFORMAT_ROOT}/bin-real/clang-format" \\
     "\$@"
EOF
chmod +x "${WORKSPACE_ROOT}/.bin/clang-format"

echo "hermetic clangd       → ${WORKSPACE_ROOT}/.bin/clangd"
echo "hermetic clang-format → ${WORKSPACE_ROOT}/.bin/clang-format"
