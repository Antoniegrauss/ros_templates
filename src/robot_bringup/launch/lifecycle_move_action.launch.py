from launch import LaunchDescription

from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    launch_description = LaunchDescription()

    node_names = ["move_robot_lifecycle_1", "move_robot_lifecycle_2"] 
    action_names = ["action_1", "action_2"]
    for name in zip(node_names, action_names):
        launch_description.add_action(
            LifecycleNode(
                package="actions_cpp",
                executable="lifecycle_move_robot_server",
                name=name[0],
                namespace="",
                parameters=[
                    {"action_name": name[1]}
                ]
            )
        )

    launch_description.add_action(
        Node(
            package="lifecycle_py",
            executable="lifecycle_move_action_manager",
            parameters=[
                {"managed_node_names": node_names}
            ]
        )
    )

    return launch_description
