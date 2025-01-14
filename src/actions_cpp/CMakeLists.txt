cmake_minimum_required(VERSION 3.8)
project(actions_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(demo_interfaces REQUIRED)


add_executable(count_until_server src/count_until_server.cpp)
target_include_directories(count_until_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
ament_target_dependencies(count_until_server rclcpp rclcpp_action demo_interfaces)

add_executable(count_until_client src/count_until_client.cpp)
target_include_directories(count_until_client PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
ament_target_dependencies(count_until_client rclcpp rclcpp_action demo_interfaces)


add_executable(lifecycle_move_robot_server src/lifecycle_move_robot_server.cpp)
target_include_directories(lifecycle_move_robot_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
ament_target_dependencies(lifecycle_move_robot_server rclcpp rclcpp_action rclcpp_lifecycle demo_interfaces)

add_executable(move_robot_server src/move_robot_server.cpp)
target_include_directories(move_robot_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
ament_target_dependencies(move_robot_server rclcpp rclcpp_action demo_interfaces)

add_executable(move_robot_client src/move_robot_client.cpp)
target_include_directories(move_robot_client PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
ament_target_dependencies(move_robot_client rclcpp rclcpp_action demo_interfaces std_msgs)


install(
  TARGETS 
  count_until_server 
  count_until_client
  move_robot_server 
  lifecycle_move_robot_server 
  move_robot_client 
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
