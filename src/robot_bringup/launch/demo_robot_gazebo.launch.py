from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution,
    PathJoinSubstitution
)

from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    world_name = LaunchConfiguration('world_name')
    world_name_launch_parameter = DeclareLaunchArgument(
        'world_name', description="Gazebo world file to use",
        default_value=TextSubstitution(text='floor_plan.world')
    )
    gazebo_world_path = os.path.join(get_package_share_path('robot_bringup'),
                             'worlds')
    gazebo_world_path = PathJoinSubstitution([gazebo_world_path, world_name])
    gazebo_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_path('gazebo_ros'),
                'launch/gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': gazebo_world_path
        }.items()
    )

    robot_name = LaunchConfiguration('robot_name')
    robot_name_launch_parameter = DeclareLaunchArgument(
        'robot_name', description="Select the robot to launch", 
        default_value=TextSubstitution(text='mobile_base_with_sensors.urdf.xacro')
    )
    urdf_path = os.path.join(get_package_share_path('demo_robot_description'),
                             'urdf')
    urdf_path = PathJoinSubstitution([urdf_path, robot_name])
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-topic', 'robot_description',
                    '-entity', 'robot']
    )
    rviz_config_path = os.path.join(get_package_share_path('demo_robot_description'),
                                    'rviz', 'mobile_base_with_sensors.rviz')
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[{'-d' + rviz_config_path}]
    )

    teleop_key = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        parameters=[{'scale_linear': 0.5}, {'scale_angular': 0.4}],
        remappings=[
            ("turtle1/cmd_vel", "/cmd_vel")
        ],
        prefix=['xterm -e'],
        output='screen'
    )

    return LaunchDescription([
        robot_name_launch_parameter,
        world_name_launch_parameter,
        gazebo_launch_file,
        robot_state_node,
        spawn_robot_node,
        rviz2_node,
        teleop_key
    ])
