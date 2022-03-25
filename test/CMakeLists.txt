ament_add_gtest(${PROJECT_NAME}_test_math_utils test_math_utils.cpp)
target_link_libraries(${PROJECT_NAME}_test_math_utils math_utils)

ament_add_gtest(${PROJECT_NAME}_test_eigen_utils test_eigen_utils.cpp)
target_link_libraries(${PROJECT_NAME}_test_eigen_utils eigen_utils)

ament_add_gtest(${PROJECT_NAME}_test_nav_utils test_nav_utils.cpp)
target_link_libraries(${PROJECT_NAME}_test_nav_utils nav_utils)

ament_add_gtest(${PROJECT_NAME}_test_heading test_heading.cpp)
target_link_libraries(${PROJECT_NAME}_test_heading nav_utils)

ament_add_gtest(${PROJECT_NAME}_test_pose test_pose.cpp)
target_link_libraries(${PROJECT_NAME}_test_pose nav_utils)
