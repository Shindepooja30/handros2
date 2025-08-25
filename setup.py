from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'handrobot_ros2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', ['rviz/handrobot_view.rviz']),
        ('share/' + package_name + '/urdf', [
            'urdf/handrobot.urdf.xacro',
            'urdf/handrobot_description.urdf.xacro',
            'urdf/handrobot.materials.xacro',
            'urdf/create_link.xacro'
        ]),
        ('share/' + package_name + '/ros2_control', ['ros2_control/handrobot.ros2_control.xacro']),
        ('share/' + package_name + '/config', ['config/handrobot_controllers.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blackhole-tejaswi',
    maintainer_email='blackhole-tejaswi@todo.todo',
    description='Hand Robot ROS2 Control Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hand_controller_node = handrobot_ros2_control.hand_controller_node:main',
            'pico_servo_bridge = handrobot_ros2_control.pico_servo_bridge:main',
        ],
    },
)
