#!/usr/bin/bash
# proudly AI-generated, human-reviewed
#
# Installs standalone binary dev tools into ~/.local/bin.
# Runs during Docker image build (no network calls at container start).
#
# Tools:
#   starpls    — Bazel/Starlark LSP
#   shfmt      — shell formatter (mirrors the version in aspect_rules_lint's lockfile)
#   buildifier — Bazel BUILD formatter (mirrors buildifier_prebuilt in tools.MODULE.bazel)

set -euo pipefail

BIN_DIR="$HOME/.local/bin"
mkdir -p "$BIN_DIR"

if [ ! -f "$BIN_DIR/starpls" ]; then
	wget -q -O "$BIN_DIR/starpls.tar.gz" \
		https://github.com/withered-magic/starpls/releases/download/v0.1.14/starpls-linux-amd64.tar.gz
	tar -xf "$BIN_DIR/starpls.tar.gz" -C "$BIN_DIR" && rm "$BIN_DIR/starpls.tar.gz"
	chmod +x "$BIN_DIR/starpls"
	echo "installed starpls"
fi

if [ ! -f "$BIN_DIR/shfmt" ]; then
	wget -q -O "$BIN_DIR/shfmt" \
		https://github.com/mvdan/sh/releases/download/v3.11.0/shfmt_v3.11.0_linux_amd64
	chmod +x "$BIN_DIR/shfmt"
	echo "installed shfmt v3.11.0"
fi

if [ ! -f "$BIN_DIR/buildifier" ]; then
	wget -q -O "$BIN_DIR/buildifier" \
		https://github.com/bazelbuild/buildtools/releases/download/v6.1.2/buildifier-linux-amd64
	chmod +x "$BIN_DIR/buildifier"
	echo "installed buildifier v6.1.2"
fi
