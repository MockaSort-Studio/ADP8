#!/usr/bin/bash
# proudly AI-generated, human-reviewed
#
# Installs hermetic Python 3.12 interpreter and pip site-packages symlink
# forest into .py_external_symlink/ at the workspace root.
#
# Run via: bazel run //:install_python
# Intended as devcontainer postCreateCommand.
# Symlinks point into Bazel's external repository cache — stable for the
# lifetime of the cache. Re-run after bazel clean --expunge.

set -euo pipefail

# --- begin runfiles.bash initialization ---
# shellcheck source=/dev/null
source "${RUNFILES_DIR:-"${BASH_SOURCE[0]}.runfiles"}/bazel_tools/tools/bash/runfiles/runfiles.bash"
# The source line above uses RUNFILES_DIR as an input path but does not
# guarantee it is exported afterward.  Bind it explicitly so -u doesn't fire.
RUNFILES_DIR="${RUNFILES_DIR:-${BASH_SOURCE[0]}.runfiles}"
# --- end runfiles.bash initialization ---

WORKSPACE_ROOT="${BUILD_WORKSPACE_DIRECTORY}"
DEST_DIR="${WORKSPACE_ROOT}/.py_external_symlink"

# Wipe previous state so stale symlinks don't accumulate across runs.
rm -rf "${DEST_DIR}"
mkdir -p "${DEST_DIR}/bin"

# Locate hermetic Python — @python_3_12//:python3 is staged in the runfiles
# under its canonical bzlmod name (rules_python++python+python_3_12_…).
# Search by path rather than hardcoding the canonical name so this survives
# platform changes and rules_python version bumps.
PYTHON_BIN=$(find "${RUNFILES_DIR}" -name "python3" -path "*/bin/python3" \
    -not -path "*/site-packages/*" 2>/dev/null | head -1)
if [[ -z "${PYTHON_BIN}" ]]; then
    echo "ERROR: hermetic python3 not found in runfiles — did @python_3_12//:python3 build?" >&2
    exit 1
fi
PYTHON_BIN="$(realpath "${PYTHON_BIN}")"

ln -sf "${PYTHON_BIN}" "${DEST_DIR}/bin/python3"
echo "hermetic python3      → ${DEST_DIR}/bin/python3"

# Derive output_base/external from the Python binary path.
# e.g. /…/output_base/external/rules_python++…/bin/python3
#   → /…/output_base/external
EXTERNAL_ROOT="${PYTHON_BIN%%/external/*}/external"

# Merge site-packages from all rules_python pip hubs.
echo "Scanning for pip hubs in ${EXTERNAL_ROOT}..."
for REPO_PATH in "${EXTERNAL_ROOT}"/rules_python++*pip*; do
    [[ -d "${REPO_PATH}" ]] || continue
    SITE_PKG="$(find "${REPO_PATH}" -maxdepth 4 -type d -name "site-packages" 2>/dev/null | head -n 1)"
    [[ -n "${SITE_PKG}" ]] || continue
    echo "merging pip hub:      $(basename "${REPO_PATH}")"
    cp -as "${SITE_PKG}"/* "${DEST_DIR}/" 2>/dev/null || true
done

echo ""
echo "IDE sync complete."
echo "  interpreter → ${DEST_DIR}/bin/python3"
echo "  packages    → ${DEST_DIR}/"
