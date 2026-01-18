load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_binary")
load("@com_github_mvukov_rules_ros2//ros2:launch.bzl", "ros2_launch")

package(default_visibility = ["//visibility:public"])

ros2_cpp_binary(
    name = "car_simulation",
    srcs = ["car_simulation.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":sim_param",
        "//applications:common_utils",
        "//core:tasks",
        "//messages:cpp_car_msgs",
        "@ros2_common_interfaces//:cpp_std_msgs",
        "@ros2_rclcpp//:rclcpp",
    ],
)

ros2_launch(
    name = "launch_sim_teleop",
    idl_deps = [
        # Add here a list of types the foxglove_bridge needs to work on.
        "@ros2_common_interfaces//:geometry_msgs",
        "@ros2_common_interfaces//:std_msgs",
        "//messages:car_msgs",
    ],
    launch_file = "launch_sim_teleop.py",
    nodes = [
        # application nodes
        "//applications/command_interfaces:actuation_interface_node",
        "//applications/egomotion:state_estimation_node",
        "//applications/teleop:teleop_node",
        # sim nodes
        ":car_simulation",
        "@com_github_mvukov_rules_ros2//third_party/foxglove_bridge:foxglove_bridge_node",
    ],
    deps = [
        "@com_github_mvukov_rules_ros2//third_party/foxglove_bridge:utils",
    ],
)

ros2_launch(
    name = "launch_sim",
    idl_deps = [
        # Add here a list of types the foxglove_bridge needs to work on.
        "@ros2_common_interfaces//:geometry_msgs",
        "@ros2_common_interfaces//:std_msgs",
        "//messages:car_msgs",
    ],
    launch_file = "launch_sim.py",
    nodes = [
        # application nodes
        "//applications/controls:control_node",
        "//applications/planning:planning_node",
        "//applications/command_interfaces:actuation_interface_node",
        "//applications/egomotion:state_estimation_node",
        # sim nodes
        ":car_simulation",
        "@com_github_mvukov_rules_ros2//third_party/foxglove_bridge:foxglove_bridge_node",
    ],
    deps = [
        "@com_github_mvukov_rules_ros2//third_party/foxglove_bridge:utils",
    ],
)

cc_library(
    name = "sim_param",
    hdrs = ["sim_param.hpp"],
)
