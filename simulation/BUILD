load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_binary")
load("@com_github_mvukov_rules_ros2//ros2:launch.bzl", "ros2_launch")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "model",
    srcs = ["robot_model/spot.xml"] + glob(["robot_model/stl/*"]),
)

# Courtesy of https://github.com/MindSpaceInc/Spot-MuJoCo-ROS2/tree/main
filegroup(
    name = "model_mujoco",
    srcs = [
        "robot_model/spot_mat.png",
        "robot_model/spot_mini.xml",
    ] + glob(["robot_model/meshes/*"]),
)

ros2_cpp_binary(
    name = "car_state",
    srcs = ["car_state.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//messages:cpp_car_msgs",
        "@ros2_common_interfaces//:cpp_std_msgs",
        "@ros2_rclcpp//:rclcpp",
    ],
)

ros2_launch(
    name = "launch_sim",
    idl_deps = [
        # Add here a list of types the foxglove_bridge needs to work on.
        #"@ros2_rcl_interfaces//:rcl_interfaces",
        #"@ros2_common_interfaces//:geometry_msgs",
        "@ros2_common_interfaces//:std_msgs",
        #"@ros2_geometry2//:tf2_msgs",
        "//messages:car_msgs",
    ],
    launch_file = "launch_sim.py",
    nodes = [
        "//simulation:car_state",
        "@com_github_mvukov_rules_ros2//third_party/foxglove_bridge:foxglove_bridge_node",
    ],
    deps = [
        "@com_github_mvukov_rules_ros2//third_party/foxglove_bridge:utils",
    ],
)
