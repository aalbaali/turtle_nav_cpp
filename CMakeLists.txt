cmake_minimum_required(VERSION 3.14)
project(turtle_nav_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
# The `include` directory is included for multiple targets, so use
# `include_directories` instead of `target_include_directories` for simplicity
include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add ros utils library
add_library(ros_utils src/ros_utils.cpp)
ament_target_dependencies(
  ros_utils
  rclcpp
  Eigen3
  geometry_msgs
)

# Add math-tools library
add_library(math_utils src/math_utils.cpp)
ament_target_dependencies(
  math_utils
)

# Add Eigen-tools library
add_library(eigen_utils src/eigen_utils.cpp)
target_link_libraries(eigen_utils math_utils)
ament_target_dependencies(
  eigen_utils
  Eigen3
)

# Add navigation utils
add_library(nav_utils src/pose.cpp src/heading.cpp src/nav_utils.cpp)
target_link_libraries(nav_utils math_utils eigen_utils ros_utils)
ament_target_dependencies(
  nav_utils
  Eigen3
  turtlesim
  geometry_msgs
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

# Estimated pose broadcaster
add_executable(dead_reckon_estimator src/dead_reckon_estimator.cpp)
target_link_libraries(dead_reckon_estimator nav_utils)
ament_target_dependencies(
  dead_reckon_estimator
  rclcpp
  turtlesim #TEMPORARY!
  geometry_msgs
  tf2
)

# Steering wheel encoder
add_executable(steering_wheel_encoder src/steering_wheel_encoder.cpp)
target_link_libraries(steering_wheel_encoder eigen_utils nav_utils)
ament_target_dependencies(
  steering_wheel_encoder
  rclcpp
  geometry_msgs
)

# Position sensor
add_executable(position_sensor src/position_sensor.cpp)
target_link_libraries(position_sensor ros_utils eigen_utils math_utils)
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
  dead_reckon_estimator
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files
install(DIRECTORY
  launch
  launch/config
  include
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(test)
endif()

if(BUILD_EXAMPLES)
  message(STATUS "Building examples")
  # Some of the examples use Matplot++ for plotting

  # Fetching Matplot++
  include(FetchContent)

  # Declare source for the contents
  fetchcontent_declare(matplotplusplus
          GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
          GIT_TAG origin/master)

  # Download repo and add it as a subdirectory
  fetchcontent_getproperties(matplotplusplus)
  if(NOT matplotplusplus_POPULATED)
      fetchcontent_populate(matplotplusplus)
      add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()

  add_subdirectory(examples)
endif()


ament_package()
