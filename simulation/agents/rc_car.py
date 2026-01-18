from simulation.src.base_entity import BaseEntity
import genesis as gs
import numpy as np


class RCCar(BaseEntity):
    def __init__(self, init_pos=(0, 0, 0.2)):
        super().__init__("rc_car")
        self.urdf_path = "simulation/rc_car_model/rc_car.urdf"
        self.init_pos = init_pos

    def add_to_scene(self, scene):
        self.rc_car_entity = scene.add_entity(
            gs.morphs.URDF(
                file=self.urdf_path,
                pos=self.init_pos,
            )
        )

        # TODO: add imu

        # now that entity is there, we can access its joints
        self.rear_wheel_idx = [
            self.rc_car_entity.get_joint(name).dofs_idx_local[0]
            for name in ["rear_left_wheel_joint", "rear_right_wheel_joint"]
        ]
        self.steering_idx = self.rc_car_entity.get_joint(
            "steering_joint"
        ).dofs_idx_local[0]

    def get_observation(self):
        # imu placeholder
        # for reference:
        #   accx, accy, accz, yaw, pitch, roll
        return [0, 0, 0, 0, 0, 0]

    def step(self, inputs):
        # for reference: inputs = [steer_torque, rear_axle_torque]
        self.rc_car_entity.control_dofs_force(np.array([inputs[0]]), self.steering_idx)
        self.rc_car_entity.control_dofs_force(
            np.array([inputs[1], inputs[1]]), self.rear_wheel_idx
        )
