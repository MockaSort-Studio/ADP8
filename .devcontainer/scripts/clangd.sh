#!/usr/bin/bash

bazel run @llvm_toolchain_llvm//:bin/clangd -- "$@"
