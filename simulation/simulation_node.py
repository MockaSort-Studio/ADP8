import rclpy
from rclpy.node import Node

from car_msgs.msg import IMURaw
from car_msgs.msg import SteeringAngleMes
from car_msgs.msg import ActuatorCommands

from simulation.src.simulation_runner import SimulationRunner
from simulation.agents.rc_car import RCCar


class SimulationNode(Node):

    def __init__(self):
        # ros2 stuff
        super().__init__("simulation_node")
        self.IMU_publisher_ = self.create_publisher(IMURaw, "imu_raw", 10)
        self.d_publisher_ = self.create_publisher(
            SteeringAngleMes, "steering_angle_measured", 10
        )
        self.subscription = self.create_subscription(
            ActuatorCommands, "actuator_cmd", self.listener_callback, 10
        )
        self.act_commands_ = ActuatorCommands()

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # genesis stuff
        rccar = RCCar()
        self.sim = SimulationRunner(agent=rccar, show_viewer=True, dt=timer_period)

    def listener_callback(self, msg):
        self.act_commands_ = msg

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

        # inputs: [dc_pwm, dc_direction, servo_angle, servo_speed]
        self.sim.step(
            inputs=[
                self.act_commands_.dc_pwm,
                self.act_commands_.dc_direction,
                self.act_commands_.servo_angle,
                self.act_commands_.servo_speed,
            ]
        )


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
