import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    package_name = 'robot_launch'
    get_package_share_directory(package_name)

    delta_odom = Node(
        package=package_name,
        executable='delta_odom',
        name='delta_odom',
        output='screen',
        arguments=[],
        parameters=[]
    )

    delta_motor = Node(
        package=package_name,
        executable='delta_motor',
        name='delta_motor',
        output='screen',
    )

    return LaunchDescription([
        delta_odom,
        delta_motor
    ])