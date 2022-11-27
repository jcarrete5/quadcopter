cmake_minimum_required(VERSION 3.18)

project(quadcopter)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)

add_compile_options(-Wall -Wextra -pedantic)

include(CTest)

add_subdirectory(flight-controller)
