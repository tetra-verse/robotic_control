cmake_minimum_required(VERSION 3.18.0 FATAL_ERROR)
project(robot_agent VERSION 0.1.0 LANGUAGES C CXX)

option(BUILD_ASSISTANT_TOOLS "Build assistant tools" OFF)
option(BUILD_UNIT_TESTS "Build unit tests" OFF)


set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/bin)

add_subdirectory(3rdparty)
add_subdirectory(depends)
add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(scripts)
add_subdirectory(tools)
