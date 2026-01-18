load("@com_github_mvukov_rules_ros2//ros2:launch.bzl", "ros2_launch")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("@sim_py_req//:requirements.bzl", "requirement")

py_binary(
    name = "simulation_node.py",
    srcs = [
        "genesis_sim_node.py",
    ],
    data = ["//simulation/rc_car_model"],
    deps = [
        requirement("genesis-world"),
        requirement("torch"),
        requirement("numpy"),
        "//simulation/src:keyboard_device",
    ],
)

ros2_launch(
    name = "launch_simFINISHME",
    idl_deps = [
        # Add here a list of types the foxglove_bridge needs to work on.
        "@ros2_common_interfaces//:geometry_msgs",
        "@ros2_common_interfaces//:std_msgs",
        "//messages:car_msgs",
    ],
    launch_file = "launch_sim.py",
    nodes = [
        # application nodes
        "//applications/command_interfaces:actuation_interface_node",
        "//applications/egomotion:state_estimation_node",
        "//applications/teleop:teleop_node",
        # sim nodes
        "//simulation:car_simulation",
        "@com_github_mvukov_rules_ros2//third_party/foxglove_bridge:foxglove_bridge_node",
    ],
    deps = [
        "@com_github_mvukov_rules_ros2//third_party/foxglove_bridge:utils",
    ],
)
