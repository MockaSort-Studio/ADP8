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

        # IMU
        self.imu = scene.add_sensor(
            gs.sensors.IMU(
                entity_idx=self.rc_car_entity.idx,
                link_idx_local=self.rc_car_entity.get_link("base_link").idx_local,
                pos_offset=(0.05, 0.0, 0.0),
                # noise parameters
                acc_cross_axis_coupling=(0.0, 0.01, 0.02),
                gyro_cross_axis_coupling=(0.03, 0.04, 0.05),
                acc_noise=(0.01, 0.01, 0.01),
                gyro_noise=(0.01, 0.01, 0.01),
                acc_random_walk=(0.001, 0.001, 0.001),
                gyro_random_walk=(0.001, 0.001, 0.001),
                delay=0.01,
                jitter=0.01,
                interpolate=True,
                # visualize
                draw_debug=True,
            )
        )

        # now that entity is there, we can access its joints
        self.rear_wheel_idx = [
            self.rc_car_entity.get_joint(name).dofs_idx_local[0]
            for name in ["rear_left_wheel_joint", "rear_right_wheel_joint"]
        ]
        self.steering_idx = self.rc_car_entity.get_joint(
            "steering_joint"
        ).dofs_idx_local[0]

    def get_observation(self):

        # IMU
        # true_data = imu.read_ground_truth() # for GT
        imu_data = self.imu.read()
        acc = imu_data.lin_acc  # tensor([ax, ay, az])
        gyro = imu_data.ang_vel  # tensor([wx, wy, wz])

        # steering angle
        d = self.rc_car_entity.get_dofs_position(self.steering_idx).item()
        d = d + np.clip(np.random.normal(0.0, 0.01), -0.03, 0.03)  # add noise

        # for reference:
        #   [
        #       accx, accy, accz,
        #       wx, wy, wz,
        #       d
        #   ]
        return [
            acc[0].item(),
            acc[1].item(),
            acc[2].item(),
            gyro[0].item(),
            gyro[1].item(),
            gyro[2].item(),
            d,
        ]

    def step(self, inputs):
        # for reference: inputs = [steer_torque, rear_axle_torque]
        self.rc_car_entity.control_dofs_force(np.array([inputs[0]]), self.steering_idx)
        self.rc_car_entity.control_dofs_force(
            np.array([inputs[1], inputs[1]]), self.rear_wheel_idx
        )
