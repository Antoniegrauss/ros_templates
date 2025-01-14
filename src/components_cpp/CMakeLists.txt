cmake_minimum_required(VERSION 3.8)
project(components_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(manual_composition
src/manual_composition.cpp
src/node1.cpp
src/node2.cpp  
)
target_include_directories(manual_composition 
  PRIVATE 
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(manual_composition rclcpp)


add_library(number_publisher_component SHARED src/number_publisher.cpp)
target_include_directories(number_publisher_component 
  PRIVATE 
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(manual_composition rclcpp)
ament_target_dependencies(number_publisher_component rclcpp rclcpp_components example_interfaces)
rclcpp_components_register_nodes(number_publisher_component "components_cpp::NumberPublisher")

install(TARGETS
  number_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  manual_composition
  DESTINATION
  lib/${PROJECT_NAME}
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
