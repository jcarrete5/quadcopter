cmake_minimum_required(VERSION 3.18)
project(quadcopter LANGUAGES NONE)

# Implicitly calls enable_testing()
# https://cmake.org/cmake/help/v3.18/command/enable_testing.html
include(CTest)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

add_subdirectory(flight-controller)
