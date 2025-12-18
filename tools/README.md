# NOTE
Currently for c/cpp we're using llvm 18.1.8 installed during container build instead of integrating llvm toolchain through bazel. 

While this is against the hermeticity of bazel environment, the devcontainer provides a further isolation of the environment.

The rationale is that fetching llvm through bzlmod takes time and space that we cannot currently afford on github actions runner and consequently causes the checks to fail due to saturation of runner filesystem.

Since we're not cross building, for now, this is fine. Over time this weakness will be addressed.