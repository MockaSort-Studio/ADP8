load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_library")

package(default_visibility = ["//visibility:public"])

ros2_cpp_library(
    name = "task_interface",
    hdrs = ["task_interface.hpp"],
    deps = [
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rclcpp//:rclcpp_components",
    ],
)
