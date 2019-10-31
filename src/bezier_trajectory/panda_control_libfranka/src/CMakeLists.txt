project(trajectory_planner)

# Dependencies
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Franka 0.6 REQUIRED)
find_package(RapidJSON REQUIRED)


add_library(${PROJECT_NAME}
  trajectory/curve_trajectory.cpp
  trajectory/linear_trajectory.cpp
  trajectory/smooth_curve_trajectory.cpp

  utility/smooth_motion_profile.cpp
  utility/curve_smoothener.cpp
  utility/trajectory_iterator_cartesian.cpp
  utility/trajectory_iterator_cartesian_velocity.cpp
  utility/cartesian_pose.cpp
  utility/kbhit.cpp
  utility/libfranka_serialize.cpp

)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC Franka::Franka)

add_executable(${PROJECT_NAME}_demo demo_curve.cpp)
target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_NAME} Franka::Franka)
target_include_directories(${PROJECT_NAME}_demo PUBLIC ${PROJECT_SOURCE_DIR})

#make executable for gripper (testing -> include as lib later)
add_executable(gripper_demo gripper_control.cpp)
target_link_libraries(gripper_demo Franka::Franka)
target_include_directories(gripper_demo PUBLIC ${PROJECT_SOURCE_DIR})