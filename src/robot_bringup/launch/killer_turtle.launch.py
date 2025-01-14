from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_description =  LaunchDescription()

    turtle_name = "Eduard"
    turtlesim_node = Node(
        package="turtlesim",
        executable = "turtlesim_node",
        remappings=[
            ("turtle1/teleport_absolute", turtle_name + "/teleport_absolute")
        ]
    )
    turtle_controller_node = Node(
        package="killer_turtle",
        executable="controller_node",
        parameters=[
            {"turtle_name": turtle_name},
            {"kill_countdown": 4}
        ]
    )
    turtle_spawner_node = Node(
        package="killer_turtle",
        executable="spawner_node",
        parameters=[
            {"spawn_cooldown": 5}
        ]
    )

    launch_description.add_action(turtlesim_node)
    launch_description.add_action(turtle_controller_node)
    launch_description.add_action(turtle_spawner_node)

    return launch_description