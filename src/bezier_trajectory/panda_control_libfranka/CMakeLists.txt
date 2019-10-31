CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

project(franka_trajectory_generation
        VERSION 1.0.0
        LANGUAGES CXX)

message("------------------------------------")
message(" ${PROJECT_NAME} ${${PROJECT_NAME}_VERSION}")
message("------------------------------------")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#if(NOT CMAKE_BUILD_TYPE) 
#    set(CMAKE_BUILD_TYPE Release)
#endif(NOT CMAKE_BUILD_TYPE)

if(NOT CMAKE_BUILD_TYPE) 
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if(MSVC)
  add_compile_options(/W0)
else()
  add_compile_options(-Wall -Wextra)
  if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    add_compile_options(-pipe) #  Avoid temporary files, speeding up builds
  endif()
endif()

option(STRICT "Treat warnings as errors" ON)
if(STRICT)
  if(MSVC)
    add_compile_options(/WX)
  else()
    add_compile_options(-Werror)
  endif()
endif()


add_subdirectory(src)
add_subdirectory(res/)
add_subdirectory(test)
add_subdirectory(documentation)
