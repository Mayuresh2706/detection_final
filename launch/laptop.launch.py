import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    nav2_params = os.path.join(
        get_package_share_directory('my_explorer'), 'config', 'nav2_params.yaml'
    )

    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_cartographer'),
                         'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_navigation2'),
                         'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'slam': 'False'
        }.items()
    )

    return LaunchDescription([
        cartographer,
        nav2,

        ExecuteProcess(
            cmd=['rviz2', '-d',
                 os.path.join(get_package_share_directory('turtlebot3_cartographer'),
                              'rviz', 'tb3_cartographer.rviz')],
            output='screen'
        ),

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
            executable='mission_manager',
            name='mission_manager',
            output='screen'
        ),

        Node(
            package='my_explorer',
            executable='run_explorer',
            name='simple_explorer',
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
