project ("libfranka_axes_motion")

find_package(Franka 0.6 REQUIRED)

add_library(${PROJECT_NAME} from_libfranka/examples_common.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} Franka::Franka)