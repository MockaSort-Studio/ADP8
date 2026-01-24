import rclpy
from rclpy.node import Node

from car_msgs.msg import IMURaw
from car_msgs.msg import SteeringAngleMes

from simulation.src.motor_models import DCMotor, ServoMotor
from simulation.src.simulation_runner import SimulationRunner
from simulation.agents.rc_car import RCCar


class SimulationNode(Node):

    def __init__(self):
        super().__init__("simulation_node")
        self.IMU_publisher_ = self.create_publisher(IMURaw, "imu_raw", 10)
        self.d_publisher_ = self.create_publisher(
            SteeringAngleMes, "steering_angle_measured", 10
        )

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        rccar = RCCar()
        self.sim = SimulationRunner(agent=rccar, show_viewer=True, dt=timer_period)

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

    def timer_callback(self):

        # noisy observations (sim outputs, imu and steering angle)
        noisy_obs_dict = self.sim.get_noisy_observations()

        IMUmsg = IMURaw()
        IMUmsg.acc_x = noisy_obs_dict["acc"][0]
        IMUmsg.acc_y = noisy_obs_dict["acc"][1]
        IMUmsg.acc_z = noisy_obs_dict["acc"][2]
        IMUmsg.gyro_x = noisy_obs_dict["gyro"][0]
        IMUmsg.gyro_y = noisy_obs_dict["gyro"][1]
        IMUmsg.gyro_z = noisy_obs_dict["gyro"][2]
        self.IMU_publisher_.publish(IMUmsg)

        d_msg = SteeringAngleMes()
        d_msg.d = noisy_obs_dict["d"][0]
        self.d_publisher_.publish(d_msg)

        # motor modelling

        # from actuators_msg
        dc_pwm = 0.8
        dc_direction = 1
        servo_angle = 10 / 180 * 3.1416
        servo_speed = 2.0
        # TODO

        obs_dict = self.sim.get_observations()
        dc_torque = self.dc_motor.step(
            dc_pwm=dc_pwm,
            dc_direction=dc_direction,
            angular_speed=obs_dict["steer_axle_w"][0],
        )

        servo_torque = self.servo_motor.step(
            servo_angle=servo_angle,
            servo_speed=servo_speed,
            current_angle=obs_dict["d"][0],
            angular_speed=obs_dict["steer_axle_w"][0],
        )

        # step with inputs [steer_torque, rear_axle_torque]
        self.sim.step(inputs=[servo_torque, dc_torque])


def main(args=None):
    rclpy.init(args=args)

    sim_node = SimulationNode()

    rclpy.spin(sim_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
