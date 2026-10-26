cmake_minimum_required(VERSION 3.8)
project(paia26_moveit)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(station_move src/station_move.cpp)
target_compile_features(station_move PUBLIC cxx_std_17)
ament_target_dependencies(station_move rclcpp moveit_ros_planning_interface)

install(TARGETS station_move DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
