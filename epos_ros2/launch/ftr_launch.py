import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    epos2_params = os.path.join(
        get_package_share_directory('epos_ros2'),
        'config',
        'ft_config.yaml'
    )

    motor_interface_node = Node(
        namespace='/ftr',
        package='epos_ros2',
        executable='epos_interface_node',
        name='epos_interface_node',
        parameters=[epos2_params]
    )

    ld.add_action(motor_interface_node)

    return ld