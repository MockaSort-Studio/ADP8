import launch
import launch_ros.actions

import third_party.foxglove_bridge.node_path
import third_party.foxglove_bridge.params


def generate_launch_description():
    """Launch the nodes."""
    return launch.LaunchDescription(
        [
            # ROS_DISTRO is necessary for correct operation of the Foxglove Studio.
            launch.actions.SetEnvironmentVariable(name="ROS_DISTRO", value="humble"),
            launch_ros.actions.Node(
                executable="simulation/car_simulation",
                output="screen",
                name="car_simulation",
            ),
            launch_ros.actions.Node(
                executable="applications/egomotion/state_estimation_node",
                output="screen",
                name="state_estimation",
            ),
            launch_ros.actions.Node(
                executable="applications/command_interfaces/actuation_interface_node",
                output="screen",
                name="actuation_interface",
            ),
            launch_ros.actions.Node(
                executable="applications/teleop/teleop_node",
                output="screen",
                name="teleop",
            ),
            launch_ros.actions.Node(
                executable=third_party.foxglove_bridge.node_path.NODE_PATH,
                output="screen",
                parameters=[
                    third_party.foxglove_bridge.params.PARAMS_TO_DEFAULT_VALUES,
                ],
            ),
        ]
    )
