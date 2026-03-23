#!/usr/bin/bash
# proudly AI-generated, human-reviewed
#
# Installs hermetic buildifier from the registered buildifier_prebuilt module
# into .bin/ at the workspace root.
#
# Run via: bazel run //:install_buildifier
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

ln -sf "$(realpath "$(rlocation buildifier_prebuilt/buildifier/buildifier)")" "${WORKSPACE_ROOT}/.bin/buildifier"

echo "hermetic buildifier → ${WORKSPACE_ROOT}/.bin/buildifier"
