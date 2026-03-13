load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dds",
    srcs = [],
    hdrs = [
        "dds_context.hpp",
        "dds_publisher.hpp",
        "dds_subscriber.hpp",
        "topic_spec.hpp",
    ],
    deps = [
        "//core/support/utils",
        "@boost.core//:core",
        "@fastdds",
    ],
)
