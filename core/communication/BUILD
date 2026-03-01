load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dds",
    srcs = [],
    hdrs = [
        "dds_publisher.hpp",
        "dds_subscriber.hpp",
        "dds_topic.hpp",
        "size_constrained_queue.hpp",
    ],
    deps = ["@fastdds"],
)
