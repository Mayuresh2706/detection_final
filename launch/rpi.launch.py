import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('turtlebot3_bringup'),
                             'launch', 'robot.launch.py')
            )
        ),

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

        # Enable motor power after turtlebot3_node has time to start
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
