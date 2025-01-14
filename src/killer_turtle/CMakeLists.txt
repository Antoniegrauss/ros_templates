cmake_minimum_required(VERSION 3.8)
project(killer_turtle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtle_msgs REQUIRED)
find_package(turtlesim REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(controller_node src/ControllerNode.cpp)
ament_target_dependencies(controller_node rclcpp turtle_msgs geometry_msgs turtlesim)

add_executable(spawner_node src/SpawnerNode.cpp)
ament_target_dependencies(spawner_node rclcpp turtle_msgs geometry_msgs turtlesim)

install(TARGETS 
  controller_node
  spawner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
