import rclpy
from rclpy.node import Node

from car_msgs.msg import IMURaw
from car_msgs.msg import SteeringAngleMes

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

    def timer_callback(self):

        # observations (sim outputs, imu and steering angle)
        obs = self.sim.get_observations()

        IMUmsg = IMURaw()
        IMUmsg.acc_x = obs[0]
        IMUmsg.acc_y = obs[1]
        IMUmsg.acc_z = obs[2]
        IMUmsg.gyro_x = obs[3]
        IMUmsg.gyro_y = obs[4]
        IMUmsg.gyro_z = obs[5]
        self.IMU_publisher_.publish(IMUmsg)

        d_msg = SteeringAngleMes()
        d_msg.d = obs[6]
        self.d_publisher_.publish(d_msg)

        # get inputs

        inputs = [0, 0]

        # step with inputs [steer_torque, rear_axle_torque]
        self.sim.step(inputs=inputs)


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
