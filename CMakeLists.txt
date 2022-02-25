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
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

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

# Install targets
install(TARGETS
  turtle_est_broadcaster
  steering_wheel_encoder
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
