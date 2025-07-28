from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='py_robot', executable='barcode_manager'),
        Node(package='py_robot', executable='door_handle_manager'),
        Node(package='py_robot', executable='emergency_button_manager'),
        Node(package='py_robot', executable='stack_light_manager'),
        Node(package='py_robot', executable='robot_manager'),
        Node(package='py_robot', executable='ui_manager')
    ])