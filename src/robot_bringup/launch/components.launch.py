from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    launch_description = LaunchDescription()

    container = ComposableNodeContainer(
        name="demo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions = [
            ComposableNode(
                package="components_cpp",
                plugin="components_cpp::NumberPublisher",
                name="num_pub_1"
            ),
            ComposableNode(
                package="components_cpp",
                plugin="components_cpp::NumberPublisher",
                name="num_pub_2"
            ),
        ]
    )

    launch_description.add_action(container)

    return launch_description