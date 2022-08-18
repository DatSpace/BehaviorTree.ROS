cmake_minimum_required(VERSION 3.16.3)
project(behaviortree_ros2)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  behaviortree_cpp_v3
  example_interfaces
  rclcpp
  rclcpp_action
)

add_executable(bt_executable
    examples/fibonacci_bt.cpp
)
target_include_directories(bt_executable PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bt_executable PUBLIC cxx_std_17)
ament_target_dependencies(bt_executable
  PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
install(
  TARGETS
    bt_executable
  ARCHIVE DESTINATION
    lib
  LIBRARY DESTINATION
    lib
  RUNTIME DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_export_include_directories(include)

ament_package()
