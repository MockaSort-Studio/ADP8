load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//core:__subpackages__"])

cc_library(
    name = "dds",
    srcs = [],
    hdrs = [
        "dds_context.hpp",
        "dds_publisher.hpp",
        "dds_subscriber.hpp",
    ],
    deps = [
        "//core/support/utils",
        "@fastdds",
    ],
)
