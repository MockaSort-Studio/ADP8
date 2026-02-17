#!/usr/bin/bash

set -eo pipefail

echo "Local host HOME: ${HOME}"
echo "Cache dictory: ${HOME}/.cache"

mkdir -p "${HOME}"/.cache/bazel && chmod 777 "${HOME}"/.cache/bazel

echo "data directory: ${HOME}/.cache/adp8_data"
mkdir -p "${HOME}"/adp8_data && chmod 777 "${HOME}"/adp8_data
