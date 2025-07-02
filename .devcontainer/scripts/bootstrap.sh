#!/usr/bin/bash

set -e

BIN_DIR=$HOME/.local/bin
WS_DIR=$1

if [ ! -d "$BIN_DIR" ]; then
    mkdir -p $BIN_DIR
fi
cd $BIN_DIR

# Download bazel buildifier
wget -O buildifier https://github.com/bazelbuild/buildtools/releases/download/v7.3.1/buildifier-linux-amd64
chmod +x buildifier

# Download bazel lsp
wget -O starpls.tar.gz https://github.com/withered-magic/starpls/releases/download/v0.1.14/starpls-linux-amd64.tar.gz
tar -xf starpls.tar.gz && rm -rf starpls.tar.gz
chmod +x starpls

chmod +x $WS_DIR/.devcontainer/scripts/clangd.sh
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
echo 'eval "$(direnv hook zsh)"' >> ~/.zshrc