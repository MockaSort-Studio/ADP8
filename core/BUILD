load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_library")

package(default_visibility = ["//visibility:public"])

ros2_cpp_library(
    name = "tasks",
    hdrs = [
        "task_interface.hpp",
        "tasks_manager.hpp",
    ],
    deps = [
        "//support:utils",
        "@boost.core",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rclcpp//:rclcpp_components",
    ],
)
