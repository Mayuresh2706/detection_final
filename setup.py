from setuptools import find_packages, setup
import os

package_name = 'aruco_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/laptop_launch_no_nav.py',
            'launch/laptop.launch.py',
            'launch/rpi.launch.py',
        ]),
        (os.path.join('lib', 'python3.10', 'site-packages', 'aruco_detection'),
            ['aruco_detection/camera_calibration.npz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mayuresh',
    maintainer_email='mayuresh2706@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'camera_node = aruco_detection.aruco_detection:main',
            'pnp_node = aruco_detection.aruco_detection_PNP:main',
            'docking_node = aruco_detection.docking_base:main',
            'task_a_node = aruco_detection.Task_A_Main:main',
            'task_b_node = aruco_detection.Task_B_Main:main',
            'mission_manager = aruco_detection.docking_main:main',
            'docking_no_nav = aruco_detection.docking_no_nav:main',
            'servo_node = aruco_detection.servo_node:main',
        ],
    },
)