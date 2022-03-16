cmake_minimum_required(VERSION 3.5)
project(turtle_nav_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

# https://github.com/ros2/eigen3_cmake_module#:~:text=Edit%20your%20CMakeLists.txt
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

#######################################################
# Add custom messages
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Vector3WithCovariance.msg"
  "msg/Vector3WithCovarianceStamped.msg"
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
#######################################################

#######################################################
# Add utils library
add_library(utils src/utils.cpp)
target_include_directories(utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
# target_include_directories(utils PRIVATE include)
ament_target_dependencies(
  utils
  rclcpp
  tf2
  tf2_ros
  turtlesim
  geometry_msgs
  Eigen3
)
#######################################################

# Estimated pose broadcaster
add_executable(turtle_est_broadcaster src/turtle_est_broadcaster.cpp)
ament_target_dependencies(
  turtle_est_broadcaster
  rclcpp
  tf2
  tf2_ros
  turtlesim
  geometry_msgs
)

# Steering wheel encoder
add_executable(steering_wheel_encoder src/steering_wheel_encoder.cpp)
ament_target_dependencies(
  steering_wheel_encoder
  rclcpp
  geometry_msgs
)

# Position sensor
add_executable(position_sensor src/position_sensor.cpp)
target_link_libraries(position_sensor utils)
ament_target_dependencies(
  position_sensor
  rclcpp
  geometry_msgs
  turtlesim
  Eigen3
)

# Add custom messages dependencies
rosidl_target_interfaces(position_sensor
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS
  turtle_est_broadcaster
  steering_wheel_encoder
  position_sensor
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files
install(DIRECTORY
  launch
  include
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(test)
endif()


ament_package()
