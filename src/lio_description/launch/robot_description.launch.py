from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_name = 'lio_description'

    pkg_path = get_package_share_directory(package_name)

    urdf_file = os.path.join(
        pkg_path,
        'urdf',
        'lio_rover.urdf'
    )

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    # Publish the URDF TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            }
        ]
    )

    # Static transform:
    # lidar_link -> livox_frame
    livox_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_static_tf',
        arguments=[
            '0', '0', '0',       # x y z
            '0', '0', '0',       # roll pitch yaw
            'lidar_link',       # parent frame
            'livox_frame'       # child frame
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        livox_static_tf
    ])
