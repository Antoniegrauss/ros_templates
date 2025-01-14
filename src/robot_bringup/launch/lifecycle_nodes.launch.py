from launch import LaunchDescription

from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    launch_description = LaunchDescription()

    number_node_name= "demo_number_publisher"
    launch_description.add_action(
        LifecycleNode(
            package="lifecycle_py",
            executable="number_publisher",
            name=number_node_name,
            namespace=""
        )
    )

    launch_description.add_action(
        Node(
            package="lifecycle_py",
            executable="lifecycle_node_manager",
            parameters=[
                {"managed_node_name": number_node_name}
            ]
        )
    )

    return launch_description
