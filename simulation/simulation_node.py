import rclpy
from rclpy.node import Node

from car_msgs.msg import IMURaw


class SimulationNode(Node):

    def __init__(self):
        super().__init__("simulation_node")
        self.publisher_ = self.create_publisher(IMURaw, "imu_raw", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = IMURaw()
        msg.acc_x = 0.0
        msg.acc_y = 10.0
        msg.acc_z = 20.0
        msg.gyro_x = 0.0
        msg.gyro_y = 0.0
        msg.gyro_z = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%i"' % self.i)
        self.i += 1


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
