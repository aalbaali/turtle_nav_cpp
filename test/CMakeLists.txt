ament_add_gtest(${PROJECT_NAME}_test_ros_utils test_ros_utils.cpp)
target_link_libraries(${PROJECT_NAME}_test_ros_utils ros_utils)

ament_add_gtest(${PROJECT_NAME}_test_eigen_utils test_eigen_utils.cpp)
target_link_libraries(${PROJECT_NAME}_test_eigen_utils eigen_utils)
