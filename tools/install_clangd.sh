#!/usr/bin/bash
# proudly AI-generated, human-reviewed
#
# Installs hermetic clangd and clang-format from the registered LLVM 19
# toolchain into .bin/ at the workspace root.
#
# Run via: bazel run //:install_clangd
# Intended as devcontainer postCreateCommand.
# Symlinks point into Bazel's external repository cache — stable for the
# lifetime of the cache. Re-run after bazel clean --expunge.

set -euo pipefail

# --- begin runfiles.bash initialization ---
# shellcheck source=/dev/null
source "${RUNFILES_DIR:-"${BASH_SOURCE[0]}.runfiles"}/bazel_tools/tools/bash/runfiles/runfiles.bash"
# --- end runfiles.bash initialization ---

WORKSPACE_ROOT="${BUILD_WORKSPACE_DIRECTORY}"
mkdir -p "${WORKSPACE_ROOT}/.bin"

# Resolve through the runfiles symlinks to the stable path inside Bazel's
# external repository cache, so .bin/ entries survive after the run exits.
ln -sf "$(realpath "$(rlocation llvm_toolchain/clangd)")"       "${WORKSPACE_ROOT}/.bin/clangd"
ln -sf "$(realpath "$(rlocation llvm_toolchain/clang-format)")" "${WORKSPACE_ROOT}/.bin/clang-format"

echo "hermetic clangd      → ${WORKSPACE_ROOT}/.bin/clangd"
echo "hermetic clang-format → ${WORKSPACE_ROOT}/.bin/clang-format"
