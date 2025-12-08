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
