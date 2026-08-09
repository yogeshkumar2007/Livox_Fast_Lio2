import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from nav_msgs.msg import Odometry


class DynamicMapFilter(Node):

    def __init__(self):
        super().__init__('dynamic_map_filter')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter(
            'cloud_topic',
            '/cloud_registered'
        )

        self.declare_parameter(
            'odom_topic',
            '/Odometry'
        )

        self.declare_parameter(
            'static_cloud_topic',
            '/static_cloud'
        )

        self.declare_parameter(
            'dynamic_cloud_topic',
            '/dynamic_cloud'
        )

        self.declare_parameter(
            'voxel_size',
            0.20
        )

        self.declare_parameter(
            'max_range',
            30.0
        )

        self.declare_parameter(
            'min_range',
            0.5
        )

        self.declare_parameter(
            'occupied_threshold',
            3
        )

        self.declare_parameter(
            'free_threshold',
            -3
        )

        self.declare_parameter(
            'decay_rate',
            1
        )

        self.declare_parameter(
            'max_voxels',
            500000
        )

        # ---------------------------------------------------------
        # Read parameters
        # ---------------------------------------------------------

        self.cloud_topic = self.get_parameter(
            'cloud_topic'
        ).value

        self.odom_topic = self.get_parameter(
            'odom_topic'
        ).value

        self.static_cloud_topic = self.get_parameter(
            'static_cloud_topic'
        ).value

        self.dynamic_cloud_topic = self.get_parameter(
            'dynamic_cloud_topic'
        ).value

        self.voxel_size = float(
            self.get_parameter(
                'voxel_size'
            ).value
        )

        self.max_range = float(
            self.get_parameter(
                'max_range'
            ).value
        )

        self.min_range = float(
            self.get_parameter(
                'min_range'
            ).value
        )

        self.occupied_threshold = int(
            self.get_parameter(
                'occupied_threshold'
            ).value
        )

        self.free_threshold = int(
            self.get_parameter(
                'free_threshold'
            ).value
        )

        self.decay_rate = int(
            self.get_parameter(
                'decay_rate'
            ).value
        )

        self.max_voxels = int(
            self.get_parameter(
                'max_voxels'
            ).value
        )

        # ---------------------------------------------------------
        # Occupancy dictionary
        #
        # voxel -> occupancy score
        #
        # positive = occupied
        # negative = free
        # ---------------------------------------------------------

        self.occupancy = {}

        # Current robot position in camera_init
        self.robot_position = np.zeros(
            3,
            dtype=np.float32
        )

        # Current robot orientation
        self.robot_rotation = np.eye(
            3,
            dtype=np.float32
        )

        self.have_odom = False

        # ---------------------------------------------------------
        # ROS publishers
        # ---------------------------------------------------------

        self.static_publisher = self.create_publisher(
            PointCloud2,
            self.static_cloud_topic,
            10
        )

        self.dynamic_publisher = self.create_publisher(
            PointCloud2,
            self.dynamic_cloud_topic,
            10
        )

        # ---------------------------------------------------------
        # ROS subscribers
        # ---------------------------------------------------------

        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'Dynamic occupancy filter started'
        )

        self.get_logger().info(
            f'Cloud: {self.cloud_topic}'
        )

        self.get_logger().info(
            f'Odometry: {self.odom_topic}'
        )

        self.get_logger().info(
            f'Static output: {self.static_cloud_topic}'
        )

        self.get_logger().info(
            f'Dynamic output: {self.dynamic_cloud_topic}'
        )

        self.get_logger().info(
            f'Voxel size: {self.voxel_size:.2f} m'
        )

    # =============================================================
    # Quaternion → Rotation Matrix
    # =============================================================

    def quaternion_to_rotation(
        self,
        x,
        y,
        z,
        w
    ):

        xx = x * x
        yy = y * y
        zz = z * z

        xy = x * y
        xz = x * z
        yz = y * z

        wx = w * x
        wy = w * y
        wz = w * z

        rotation = np.array([
            [
                1.0 - 2.0 * (yy + zz),
                2.0 * (xy - wz),
                2.0 * (xz + wy)
            ],
            [
                2.0 * (xy + wz),
                1.0 - 2.0 * (xx + zz),
                2.0 * (yz - wx)
            ],
            [
                2.0 * (xz - wy),
                2.0 * (yz + wx),
                1.0 - 2.0 * (xx + yy)
            ]
        ], dtype=np.float32)

        return rotation

    # =============================================================
    # Odometry callback
    # =============================================================

    def odom_callback(self, msg):

        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y
        self.robot_position[2] = msg.pose.pose.position.z

        q = msg.pose.pose.orientation

        self.robot_rotation = (
            self.quaternion_to_rotation(
                q.x,
                q.y,
                q.z,
                q.w
            )
        )

        self.have_odom = True

    # =============================================================
    # Convert point to voxel
    # =============================================================

    def point_to_voxel(self, point):

        voxel = np.floor(
            point / self.voxel_size
        ).astype(np.int32)

        return tuple(voxel)

    # =============================================================
    # Update occupied voxel
    # =============================================================

    def mark_occupied(self, voxel):

        current = self.occupancy.get(
            voxel,
            0
        )

        current += 1

        if current > self.occupied_threshold:
            current = self.occupied_threshold

        self.occupancy[voxel] = current

    # =============================================================
    # Update free voxel
    # =============================================================

    def mark_free(self, voxel):

        current = self.occupancy.get(
            voxel,
            0
        )

        current -= self.decay_rate

        if current < self.free_threshold:
            current = self.free_threshold

        self.occupancy[voxel] = current

    # =============================================================
    # Approximate ray clearing
    # =============================================================

    def clear_ray(
        self,
        origin,
        endpoint
    ):

        direction = endpoint - origin

        distance = np.linalg.norm(
            direction
        )

        if distance < self.min_range:
            return

        if distance > self.max_range:
            endpoint = (
                origin
                +
                direction
                *
                (self.max_range / distance)
            )

            direction = endpoint - origin
            distance = self.max_range

        steps = max(
            1,
            int(distance / self.voxel_size)
        )

        # Limit computation for very long rays
        steps = min(
            steps,
            150
        )

        for i in range(1, steps):

            ratio = float(i) / float(steps)

            point = (
                origin
                +
                direction
                *
                ratio
            )

            voxel = self.point_to_voxel(
                point
            )

            self.mark_free(voxel)

    # =============================================================
    # Point cloud callback
    # =============================================================

    def cloud_callback(self, msg):

        if not self.have_odom:
            return

        # ---------------------------------------------------------
        # Read XYZ
        # ---------------------------------------------------------

        points_struct = point_cloud2.read_points(
            msg,
            field_names=(
                'x',
                'y',
                'z'
            ),
            skip_nans=True
        )

        if len(points_struct) == 0:
            return

        points = np.column_stack((
            points_struct['x'],
            points_struct['y'],
            points_struct['z']
        )).astype(np.float32)

        if points.shape[0] == 0:
            return

        # ---------------------------------------------------------
        # Remove invalid points
        # ---------------------------------------------------------

        valid = np.isfinite(
            points
        ).all(axis=1)

        points = points[valid]

        if points.shape[0] == 0:
            return

        # ---------------------------------------------------------
        # Remove points beyond max range
        # ---------------------------------------------------------

        ranges = np.linalg.norm(
            points,
            axis=1
        )

        valid_range = (
            (ranges >= self.min_range)
            &
            (ranges <= self.max_range)
        )

        points = points[valid_range]

        if points.shape[0] == 0:
            return

        # ---------------------------------------------------------
        # cloud_registered is already in camera_init.
        #
        # Therefore the LiDAR/robot position used as the
        # ray origin is the FAST-LIO2 odometry position.
        # ---------------------------------------------------------

        origin = self.robot_position.copy()

        # ---------------------------------------------------------
        # Process every point
        # ---------------------------------------------------------

        for point in points:

            endpoint = point

            # Clear the space between sensor and obstacle
            self.clear_ray(
                origin,
                endpoint
            )

            # Endpoint is occupied
            voxel = self.point_to_voxel(
                endpoint
            )

            self.mark_occupied(
                voxel
            )

        # ---------------------------------------------------------
        # Prevent unlimited map growth
        # ---------------------------------------------------------

        if len(self.occupancy) > self.max_voxels:

            sorted_voxels = sorted(
                self.occupancy.items(),
                key=lambda item: abs(item[1]),
                reverse=True
            )

            self.occupancy = dict(
                sorted_voxels[:self.max_voxels]
            )

        # ---------------------------------------------------------
        # Generate static and dynamic clouds
        # ---------------------------------------------------------

        static_points = []
        dynamic_points = []

        for voxel, score in self.occupancy.items():

            center = (
                np.array(voxel, dtype=np.float32)
                +
                0.5
            ) * self.voxel_size

            if score >= self.occupied_threshold:

                static_points.append(
                    center.tolist()
                )

            elif score > 0:

                dynamic_points.append(
                    center.tolist()
                )

        # ---------------------------------------------------------
        # Publish static cloud
        # ---------------------------------------------------------

        if len(static_points) > 0:

            static_msg = (
                point_cloud2.create_cloud_xyz32(
                    msg.header,
                    static_points
                )
            )

            self.static_publisher.publish(
                static_msg
            )

        # ---------------------------------------------------------
        # Publish dynamic candidate cloud
        # ---------------------------------------------------------

        if len(dynamic_points) > 0:

            dynamic_msg = (
                point_cloud2.create_cloud_xyz32(
                    msg.header,
                    dynamic_points
                )
            )

            self.dynamic_publisher.publish(
                dynamic_msg
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = DynamicMapFilter()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
