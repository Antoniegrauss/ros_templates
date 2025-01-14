from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_description = LaunchDescription()

    number_publisher_node = Node(
        package="demo_cpp_package",
        executable="number_publisher",
        name="renamed_number_publisher",
        remappings=[
            ("number", "renamed_number")
        ]
    )

    number_counter_node = Node(
        package="demo_python_package",
        executable="number_counter",
        name="renamed_number_counter",
        remappings=[
            ("number", "renamed_number")
        ]
    )

    launch_description.add_action(number_publisher_node)
    launch_description.add_action(number_counter_node)


    return launch_description
