import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuUnitConverter(Node):

    def __init__(self):
        super().__init__('imu_unit_converter')

        self.G = 9.80665

        self.subscription = self.create_subscription(
            Imu,
            '/livox/imu',
            self.imu_callback,
            10
        )

        self.publisher = self.create_publisher(
            Imu,
            '/fastlio/imu',
            10
        )

        self.get_logger().info(
            'IMU converter started: acceleration g -> m/s^2'
        )

    def imu_callback(self, msg):

        output = Imu()

        output.header = msg.header

        output.orientation = msg.orientation
        output.orientation_covariance = msg.orientation_covariance

        # Gyroscope already assumed to be rad/s.
        output.angular_velocity = msg.angular_velocity
        output.angular_velocity_covariance = (
            msg.angular_velocity_covariance
        )

        # Convert acceleration from g to m/s^2.
        output.linear_acceleration.x = (
            msg.linear_acceleration.x * self.G
        )

        output.linear_acceleration.y = (
            msg.linear_acceleration.y * self.G
        )

        output.linear_acceleration.z = (
            msg.linear_acceleration.z * self.G
        )

        output.linear_acceleration_covariance = (
            msg.linear_acceleration_covariance
        )

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)

    node = ImuUnitConverter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
