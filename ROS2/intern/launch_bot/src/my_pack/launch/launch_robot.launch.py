from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'my_pack'
    package_dir = get_package_share_directory(package_name)
    script_path = os.path.join(package_dir, 'src')  

    odom_node = Node(
        package=package_name,
        executable='delta_odom.py',
        name='odom_node',
        output='screen',
        parameters=[],
        arguments=[],
        prefix='python3 ' + os.path.join(script_path, 'delta_odom.py')
    )

    motor_node = Node(
        package=package_name,
        executable='delta_motor.py',
        name='motor_node',
        output='screen',
        prefix='python3 ' + os.path.join(script_path, 'delta_motor.py')
    )

    return LaunchDescription([
        odom_node,
        motor_node
    ])
