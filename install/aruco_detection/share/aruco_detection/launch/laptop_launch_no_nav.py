from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Broadcast camera_link relative to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf',
            arguments=['0.04', '0.08', '0.15', '0', '0', '0', 'base_link', 'camera_link']
        ),

        Node(
            package='aruco_detection',
            executable='pnp_node',
            name='pnp_node',
            output='screen'
        ),

        Node(
            package='aruco_detection',
            executable='docking_no_nav',
            name='docking_no_nav',
            output='screen'
        ),

        Node(
            package='aruco_detection',
            executable='docking_node',
            name='docking_node',
            output='screen'
        ),

        Node(
            package='aruco_detection',
            executable='task_a_node',
            name='task_a_node',
            output='screen'
        ),

        Node(
            package='aruco_detection',
            executable='task_b_node',
            name='task_b_node',
            output='screen'
        ),
    ])
