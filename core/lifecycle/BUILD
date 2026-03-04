load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//core:__subpackages__"])

cc_library(
    name = "lifecycle",
    srcs = ["execution_engine.cpp"],
    hdrs = [
        "execution_engine.hpp",
        "task_interface.hpp",
        "tasks_manager.hpp",
    ],
    deps = [
        "//core/lifecycle:parameters_provider",
    ],
)

cc_library(
    name = "parameters_provider",
    hdrs = [
        "parameters_provider.hpp",
    ],
    deps = [
        "//core/support/utils",
    ],
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
