cmake_minimum_required(VERSION 3.18)

project(quadcopter)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_MODULE_PATH cmake/)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -pedantic)

add_subdirectory(flight-controller)
