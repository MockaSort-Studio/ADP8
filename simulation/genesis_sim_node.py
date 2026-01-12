import genesis as gs
import numpy
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
            pos=(0.5, 0.0, 0.07),
            size=(0.04, 0.04, 0.04),
        ),
        surface=gs.surfaces.Default(color=(0.5, 1, 0.5)),
    )

    entities["rc_car"] = scene.add_entity(
        material=gs.materials.Rigid(gravity_compensation=1),
        morph=gs.morphs.MJCF(
            file="simulation/rc_car_model/rc_car_model.xml",
            pos=(0, 0, 0),
            euler=(0, 0, 0),
        ),
    )

    # build
    scene.build()
    return scene, entities


def run_sim(scene, entities, clients):

    rc_car = entities["rc_car"]
    rc_car_mjcf = rc_car.morph  # this is the MJCF morph

    # rc_car_jnt_names = [
    #     "rl_wheel",
    #     "rr_wheel",
    # ]
    # dofs_idx = [rc_car.get_joint(name).dof_idx_local for name in rc_car_jnt_names]

    rear_torque = 0.2  # dummy value to check genesis physics

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

        # if "KEY_W" in pressed_keys:
        #     rc_car_mjcf.set_joint_torque("rl_wheel", rear_torque)
        #     rc_car_mjcf.set_joint_torque("rr_wheel", rear_torque)

        scene.step()


def main():
    clients = dict()
    clients["keyboard"] = KeyboardDevice()
    clients["keyboard"].start()

    scene, entities = build_scene()
    run_sim(scene, entities, clients)


if __name__ == "__main__":
    main()
