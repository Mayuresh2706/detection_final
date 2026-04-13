from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='aruco_detection',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),

        Node(
            package='aruco_detection',
            executable='servo_node',
            name='servo_node',
            output='screen'
        ),

        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/motor_power',
                         'std_srvs/srv/SetBool', '{data: true}'],
                    output='screen'
                )
            ]
        ),
    ])
