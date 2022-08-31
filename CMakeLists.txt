cmake_minimum_required(VERSION 3.18)

project(quadcopter)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_MODULE_PATH cmake/)

add_compile_options(-Wall -Wextra -pedantic)
add_subdirectory(flight-controller)
