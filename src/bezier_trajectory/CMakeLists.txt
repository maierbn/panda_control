cmake_minimum_required(VERSION 3.13)

# ----------------------
project(smooth_curve)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Franka 0.6 REQUIRED)

add_subdirectory(panda_control_libfranka)

include_directories(PUBLIC panda_control_libfranka/src)

add_executable(${PROJECT_NAME} 
  main_smooth_curve.cpp
  utility/trajectory_plotter.cpp
  trajectory/bezier_trajectory.cpp
)
target_link_libraries(${PROJECT_NAME} trajectory_planner Franka::Franka)
target_include_directories(${PROJECT_NAME} PUBLIC trajectory)

option(PROFILE "Add flags to profile the program with gprof." OFF)
if(PROFILE)
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pg -g")
  SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -pg -g")
endif()


# -------------------------
project(peg_in_hole)
add_executable(${PROJECT_NAME}
  main_peg_in_hole.cpp 
  trajectory/peg_in_hole_trajectory.cpp
  trajectory/bezier_trajectory.cpp
)
target_link_libraries(${PROJECT_NAME} trajectory_planner Franka::Franka)
target_include_directories(${PROJECT_NAME} PUBLIC trajectory)


# -------------------------
project(bezier)
add_executable(${PROJECT_NAME} 
  main_bezier.cpp 
  trajectory/peg_in_hole_trajectory.cpp
  trajectory/bezier_trajectory.cpp
)
target_link_libraries(${PROJECT_NAME} trajectory_planner Franka::Franka)
target_include_directories(${PROJECT_NAME} PUBLIC trajectory)
