import genesis as gs
import numpy as np
import threading
from evdev import InputDevice, categorize, ecodes
import os
from simulation.src.keyboard_device import (
    KeyboardDevice,
)

# example reference for keyboard teleop: https://github.com/Genesis-Embodied-AI/Genesis/blob/main/examples/keyboard_teleop.py


def build_scene():
    # init
    gs.init(precision="32", logging_level="warning", backend=gs.cpu)

    # scene
    scene = gs.Scene(
        show_viewer=True,
        sim_options=gs.options.SimOptions(dt=0.01, gravity=(0, 0, -9.81)),
    )

    # entities
    entities = dict()
    entities["plane"] = scene.add_entity(gs.morphs.Plane())

    entities["cube"] = scene.add_entity(
        material=gs.materials.Rigid(rho=300),
        morph=gs.morphs.Box(
            pos=(0.0, 0.0, 0.0),
            size=(0.05, 0.05, 0.05),
        ),
        surface=gs.surfaces.Default(color=(0.5, 1, 0.5)),
    )

    entities["rc_car"] = scene.add_entity(
        gs.morphs.URDF(
            file="simulation/rc_car_model/roscar.urdf",
            pos=(1, 1, 0.2),
        )
    )

    # 'back_right_wheel_link'
    # steer_joint
    # entities["imu"] = scene.add_sensor(
    #     gs.sensors.IMU(
    #         link=entities["rc_car"].get_link("base_link"),
    #     )
    # )

    # entities["cam"] = scene.add_camera(
    #     res=(640, 480),
    #     pos=(0, 0, 4),
    #     lookat=(0, 0, 0.5),
    # )

    # build
    scene.build()
    return scene, entities


def run_sim(scene, entities, clients):

    rc_car = entities["rc_car"]
    rear_wheel_joints_idx = [
        rc_car.get_joint(name).dofs_idx_local[0]
        for name in ["back_left_wheel_joint", "back_right_wheel_joint"]
    ]
    steering_joint_idx = rc_car.get_joint("steer_joint").dofs_idx_local[0]

    # rear_torque = 0.2  # dummy value to check genesis physics

    # keyoard controls
    kb_device = clients["keyboard"]
    print("\nKeyboard Controls:")
    print("esc\t- Quit")

    # start sim
    stop = False
    while not stop:
        pressed_keys = kb_device.get_cmd()
        if "KEY_ESC" in pressed_keys:
            print("Exiting...")
            stop = True

        # if "KEY_P" in pressed_keys:
        #     imu_data = entities["imu"].get_data()
        #     print(f"accel: {imu_data.linear_acceleration}")
        #     print(f"accel: {imu_data.angular_velocity}")

        if "KEY_LEFT" in pressed_keys:
            rc_car.control_dofs_force(np.array([5]), steering_joint_idx)
        if "KEY_RIGHT" in pressed_keys:
            rc_car.control_dofs_force(np.array([-5]), steering_joint_idx)

        if "KEY_UP" in pressed_keys:
            rc_car.control_dofs_force(np.array([1, 1]), rear_wheel_joints_idx)
        else:
            rc_car.control_dofs_force(np.array([0, 0]), rear_wheel_joints_idx)

        if "KEY_DOWN" in pressed_keys:
            rc_car.control_dofs_force(np.array([-1, -1]), rear_wheel_joints_idx)
        else:
            rc_car.control_dofs_force(np.array([0, 0]), rear_wheel_joints_idx)

        # step
        scene.step()


def main():
    clients = dict()
    clients["keyboard"] = KeyboardDevice()
    clients["keyboard"].start()

    scene, entities = build_scene()
    run_sim(scene, entities, clients)


if __name__ == "__main__":
    main()
