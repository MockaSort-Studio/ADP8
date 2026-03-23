#!/usr/bin/bash

set -eo pipefail

echo "Local host HOME: ${HOME}"
echo "Cache dictory: ${HOME}/.cache"

mkdir -p "${HOME}"/.cache/bazel && chmod 777 "${HOME}"/.cache/bazel

echo "data directory: ${HOME}/.cache/adp8_data"
mkdir -p "${HOME}"/adp8_data && chmod 777 "${HOME}"/adp8_data

# Pull the RBE base image so the devcontainer build uses the cached layer
# instead of downloading ~500 MB of Python deps from scratch.
# Falls back to a local build if GHCR is unreachable (offline, no auth, etc.).
RBE_IMAGE="ghcr.io/mockasort-studio/adp8-rbe:latest"
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "Pulling RBE base image ${RBE_IMAGE}..."
if ! docker pull "${RBE_IMAGE}" 2>/dev/null; then
    echo "Pull failed — building RBE image locally (this will take a while)..."
    docker build \
        -f "${WORKSPACE_DIR}/.devcontainer/Dockerfile.rbe" \
        -t "${RBE_IMAGE}" \
        "${WORKSPACE_DIR}/.devcontainer"
fi
