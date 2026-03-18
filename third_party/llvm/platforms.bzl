"""
Platform configurations for the hermetic LLVM toolchain.

To add a platform: add one entry to LLVM_PLATFORMS. No logic changes needed.

Fields per platform:
  llvm_url / llvm_sha256 / llvm_strip_prefix — LLVM release tarball
  sysroot_url / sysroot_sha256               — Chromium sysroot (TARGET)
  ld_interp                                  — dynamic linker filename
  host_runtime_debs                          — Bookworm debs for HOST runtime
  sysroot_libs                               — extra .so files copied from the
                                               sysroot into lib/ (not in the debs)

proudly AI-generated, human-reviewed
"""

LLVM_PLATFORMS = {
    "linux-x86_64": struct(
        llvm_url = "https://github.com/llvm/llvm-project/releases/download/llvmorg-19.1.7/LLVM-19.1.7-Linux-X64.tar.xz",
        llvm_sha256 = "4a5ec53951a584ed36f80240f6fbf8fdd46b4cf6c7ee87cc2d5018dc37caf679",
        llvm_strip_prefix = "LLVM-19.1.7-Linux-X64",

        # Chromium Bullseye sysroot — compiled binaries need glibc >= 2.31.
        # Chromium does not publish a Bookworm sysroot; Bullseye is fine since
        # any modern deployment target (Ubuntu 22.04+, Debian 12) has glibc >= 2.31.
        sysroot_url = "https://commondatastorage.googleapis.com/chrome-linux-sysroot/toolchain/2028cdaf24259d23adcff95393b8cc4f0eef714b/debian_bullseye_amd64_sysroot.tar.xz",
        sysroot_sha256 = "1be60e7c456abc590a613c64fab4eac7632c81ec6f22734a61b53669a4407346",

        # Dynamic linker name — different on aarch64 ("ld-linux-aarch64.so.1").
        ld_interp = "ld-linux-x86-64.so.2",

        # Bookworm (glibc 2.36) debs bundled in lib/ so clang can execute on
        # any Linux executor regardless of the system glibc version.
        # LLVM 19 needs symbols up to GLIBC_2.34; Bookworm satisfies that.
        host_runtime_debs = [
            struct(
                name = "libc6.deb",
                url = "https://ftp.debian.org/debian/pool/main/g/glibc/libc6_2.36-9+deb12u13_amd64.deb",
                sha256 = "3d8072c73b017e907bbf44b7db870687888a991961d74f1ecbba6b9458f32a2c",
                extractions = [
                    ("./lib/x86_64-linux-gnu/libc.so.6",            "lib/libc.so.6"),
                    ("./lib/x86_64-linux-gnu/libm.so.6",            "lib/libm.so.6"),
                    ("./lib/x86_64-linux-gnu/ld-linux-x86-64.so.2", "lib/ld-linux-x86-64.so.2"),
                ],
            ),
            struct(
                name = "libstdcxx6.deb",
                url = "https://ftp.debian.org/debian/pool/main/g/gcc-12/libstdc++6_12.2.0-14+deb12u1_amd64.deb",
                sha256 = "5cd3171216d4ab0fc911cfe9c35509bf2dd8f47761c43b7f6a4296701551a24d",
                extractions = [
                    ("./usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", "lib/libstdc++.so.6"),
                ],
            ),
            struct(
                name = "libgcc-s1.deb",
                url = "https://ftp.debian.org/debian/pool/main/g/gcc-12/libgcc-s1_12.2.0-14+deb12u1_amd64.deb",
                sha256 = "3016e62cb4b7cd8038822870601f5ed131befe942774d0f745622cc77d8a88f7",
                extractions = [
                    ("./lib/x86_64-linux-gnu/libgcc_s.so.1", "lib/libgcc_s.so.1"),
                ],
            ),
            # lld links against libxml2 (for MachO TBD parsing); libxml2 in turn
            # pulls ICU and liblzma.  All from Bookworm to stay within glibc 2.36.
            struct(
                name = "liblzma5.deb",
                url = "https://ftp.debian.org/debian/pool/main/x/xz-utils/liblzma5_5.4.1-1_amd64.deb",
                sha256 = "d321b9502b16aac534e1c691afbe3dc5e125e5091aa35bea026c59b25ebe82e7",
                extractions = [
                    ("./lib/x86_64-linux-gnu/liblzma.so.5.4.1", "lib/liblzma.so.5"),
                ],
            ),
            struct(
                name = "libicu72.deb",
                url = "https://ftp.debian.org/debian/pool/main/i/icu/libicu72_72.1-3+deb12u1_amd64.deb",
                sha256 = "f7f6f99c6d7b025914df2447fc93e11d22c44c0c8bdd8b6f36691c9e7ddcef88",
                extractions = [
                    ("./usr/lib/x86_64-linux-gnu/libicuuc.so.72.1",   "lib/libicuuc.so.72"),
                    ("./usr/lib/x86_64-linux-gnu/libicudata.so.72.1", "lib/libicudata.so.72"),
                ],
            ),
            struct(
                name = "libxml2.deb",
                url = "https://ftp.debian.org/debian/pool/main/libx/libxml2/libxml2_2.9.14+dfsg-1.3~deb12u5_amd64.deb",
                sha256 = "261ad120adf02d9bdef3f47c486e016bfd410cdb1ada9730a8f9a6add0857bd6",
                extractions = [
                    ("./usr/lib/x86_64-linux-gnu/libxml2.so.2.9.14", "lib/libxml2.so.2"),
                ],
            ),
        ],

        # libs present in the sysroot but not covered by the debs above.
        # (src_dir_in_sysroot, glob_pattern, dest_in_lib)
        sysroot_libs = [
            ("usr/lib/x86_64-linux-gnu", "libzstd.so.1.*", "lib/libzstd.so.1"),
            ("lib/x86_64-linux-gnu",     "libz.so.1.*",    "lib/libz.so.1"),
        ],
    ),
    # ── linux-aarch64 ──────────────────────────────────────────────────────────
    # Uncomment and fill URLs/sha256 to enable arm64 support.
    # "linux-aarch64": struct(
    #     llvm_url          = "https://github.com/llvm/llvm-project/releases/download/llvmorg-19.1.7/LLVM-19.1.7-Linux-ARM64.tar.xz",
    #     llvm_sha256       = "TODO",
    #     llvm_strip_prefix = "LLVM-19.1.7-Linux-ARM64",
    #     sysroot_url       = "TODO",
    #     sysroot_sha256    = "TODO",
    #     ld_interp         = "ld-linux-aarch64.so.1",
    #     host_runtime_debs = [...],  # same structure, _arm64.deb variants
    #     sysroot_libs      = [
    #         ("usr/lib/aarch64-linux-gnu", "libzstd.so.1.*", "lib/libzstd.so.1"),
    #         ("lib/aarch64-linux-gnu",     "libz.so.1.*",    "lib/libz.so.1"),
    #     ],
    # ),
}
