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
        "dds_application.hpp",
        "dds_task.hpp",
    ],
    deps = [
        ":lifecycle",
        "//core/communication:dds",
    ],
)
