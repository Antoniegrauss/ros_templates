# ROS Templates

This project contains templates and setup to get a ROS project up and running quickly.
The packages in src contain examples for simple and more advanced use-cases.

The templates are based upon ROS2 humble.

Most examples can be run through a launch file in the `robot_bringup` package.
Use the `ros2 launch` together with `--show-args` to find out which launch
arguments are supported.

The `killer_turtle` is not a standalone template, but more a simple project on how
to create a simple controller and connect it via ROS.

The `robot_bringup demo_robot_gazebo.launch.py` runs a full mobile robot project
where you can drive around a simple differential drive robot in gazebo. The robot
also features simulated 2D lidar and Camera sensors.

Further packages in this repository are:

- demo_nodes_cpp/demo_nodes_py: examples on ROS nodes, publishing, subscribing,
service server/client (also with custom messages/services)
- demo_services: example of a simple project with services.
- demo_interfaces: custom msg,srv,action types and CMakeLists.txt to build them
- actions_cpp/actions_py: action service/client, also with multithreading and callback groups,
strategies for multiple goal handling, goal canceling and feedback topics.
- demo_robot_description: urdf and xacro for robot building. Contains xacro macro's,
gazebo plugins for sensors, diff drive and robot arm control, rviz configuration file.
- executors_cpp/py: examples of when a MultiThreadedExecutor is necessary to process multiple
callback in parralel. (TODO: fix the main to spin up nodes)
- components_cpp/py: Shows manual composition for nodes. (As opposed to run-time composition
via the command line)
- lifecycle_cpp/py: lifecycle node templates, including functions for state changes and
a lifecycle_manager node that manages the state changes.
- killer_turtle: example project on how to write a simple controller and integrate it into
the turtle_sim package
- turtle_msgs: messages for the killer_turtle package.

## Run with Docker Compose

To be able to run these examples you need ROS2 humble and some other libraries.
Alternatively you can use the docker compose file to launch a container with
a terminator terminal and run the code there.

To run simply build and run the container:


```bash
docker compose build
docker compose up

```

Then in the new terminal source the compiled packages and run a project:

```bash
source install/setup.bash
ros2 launch robot_bringup ...
```

Use `tab tab` to see the options for ros2 launch.
If no options show, try sourcing the workspace, sourcing ros (`source /opt/ros/humble/setup.bash`)
or compiling the workspace:

```bash
cd ros_templates
colcon build
```

## Vscode Setup Files

The files in the `.vscode` directory are setup files for VS Code to help the intellisense
and configure the ROS2 extension.
The `settings.json` shows how to add packages to the python autocomplete as well. Add more
paths for packages if you need autocomplete for them here.
The `c_cpp_properties.json` shows how to add paths for the c++ autocomplete. 

Note that adding the whole workspace is possible, but slows down the intellisense a lot.
