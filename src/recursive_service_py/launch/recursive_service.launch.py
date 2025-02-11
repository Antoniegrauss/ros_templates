from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = "recursive_service_py"
    return LaunchDescription([
        Node(
            package=package_name,
            executable="inner_server",
            name="inner_server"
        ),
        Node(
            package=package_name,
            executable="outer_server",
            name="outer_server"
        ),
        Node(
            package=package_name,
            executable="outer_client",
            name="outer_client"
        )
    ])
    