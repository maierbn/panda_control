project(unittests)

file(DOWNLOAD https://raw.githubusercontent.com/catchorg/Catch2/master/single_include/catch2/catch.hpp
             ${PROJECT_SOURCE_DIR}/catch.hpp TLS_VERIFY ON)

find_package(Eigen3 REQUIRED NO_MODULE)

set(TEST_SOURCES test_motion_gen/TestTrajectory.cpp test_motion_gen/TestIterator.cpp 
    test_motion_gen/TestStablePoseConversion.cpp)

add_executable(${PROJECT_NAME} TestMain.cpp ${TEST_SOURCES})
target_link_libraries(${PROJECT_NAME} trajectory_planner)



add_custom_target(tests ${PROJECT_BINARY_DIR}/${PROJECT_NAME})
add_dependencies(tests ${PROJECT_NAME})