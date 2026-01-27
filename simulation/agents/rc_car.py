import genesis as gs
import numpy as np

from simulation.src.base_entity import BaseEntity
from simulation.src.motor_models import DCMotor, ServoMotor


class RCCar(BaseEntity):
    def __init__(self, init_pos=(0, 0, 0.2)):
        super().__init__("rc_car")
        self.urdf_path = "simulation/rc_car_model/rc_car.urdf"
        self.init_pos = init_pos

        # motors init
        self.dc_motor = DCMotor(
            torque_constant=0.05,
            back_emf_constant=0.05,
            resistance=0.6,
            supply_voltage=7.4,
        )
        self.servo_motor = ServoMotor(
            max_torque=0.2, position_gain=6.0, damping_gain=0.05
        )

    def add_to_scene(self, scene) -> None:
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
                # acc_random_walk=(0.001, 0.001, 0.001),
                # gyro_random_walk=(0.001, 0.001, 0.001),
                # delay=0.01,
                # jitter=0.01,
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

    def get_observation(self) -> dict:
        # TODO: get pos and orientation as groudtruth
        # maybe also get true_imu data ?
        # true_imu = self.imu.read_ground_truth()
        d = self.rc_car_entity.get_dofs_position(self.steering_idx).item()
        steer_axle_w = self.rc_car_entity.get_dofs_velocity(self.steering_idx).item()

        # rear axle speed (theoretically left and right wheels should have the same speed if no slips or somethign weird)
        rl_w = self.rc_car_entity.get_dofs_velocity(self.rear_wheel_idx[0]).item()
        rr_w = self.rc_car_entity.get_dofs_velocity(self.rear_wheel_idx[1]).item()
        rear_axle_w = (rl_w + rr_w) / 2
        # TODO: improve simulation/rc_car_model/rc_car.urdf to have a single axle

        return {
            "steer_axle_w": [steer_axle_w],
            "d": [d],
            "rear_axle_w": [rear_axle_w],
        }

    def get_noisy_observation(self) -> dict:

        # IMU
        imu_data = self.imu.read()
        acc = imu_data.lin_acc  # tensor([ax, ay, az])
        gyro = imu_data.ang_vel  # tensor([wx, wy, wz])

        # steering angle
        d = self.rc_car_entity.get_dofs_position(self.steering_idx).item()
        d = d + np.clip(np.random.normal(0.0, 0.01), -0.03, 0.03)  # add noise

        return {
            "acc": [
                acc[0].item(),
                acc[1].item(),
                acc[2].item(),
            ],
            "gyro": [
                gyro[0].item(),
                gyro[1].item(),
                gyro[2].item(),
            ],
            "d": [d],
        }

    def step(self, inputs) -> None:
        # for reference:
        #       inputs: [dc_pwm, dc_direction, servo_angle, servo_speed]

        # motor modelling
        obs_dict = self.get_observation()
        dc_torque = self.dc_motor.step(
            dc_pwm=inputs[0],
            dc_direction=inputs[1],
            angular_speed=obs_dict["steer_axle_w"][0],
        )

        servo_torque = self.servo_motor.step(
            servo_angle=inputs[2],
            servo_speed=inputs[3],
            current_angle=obs_dict["d"][0],
            angular_speed=obs_dict["steer_axle_w"][0],
        )

        self.rc_car_entity.control_dofs_force(
            np.array([servo_torque]), self.steering_idx
        )
        self.rc_car_entity.control_dofs_force(
            np.array([dc_torque, dc_torque]), self.rear_wheel_idx
        )
