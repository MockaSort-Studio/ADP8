load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//core/lifecycle:__subpackages__"])

cc_library(
    name = "lifecycle",
    srcs = ["execution_engine.cpp"],
    hdrs = [
        "execution_engine.hpp",
        "task_interface.hpp",
        "tasks_manager.hpp",
    ],
    deps = [],
)

cc_library(
    name = "dds",
    hdrs = [
        "data_endpoint.hpp",
        "dds_application.hpp",
        "dds_task.hpp",
        "input_source.hpp",
        "output_sink.hpp",
        "type_traits.hpp",
    ],
    deps = [
        ":lifecycle",
        "//core/communication:dds",
        "//core/support/utils",
        "@boost.core",
    ],
)
