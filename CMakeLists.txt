#####  PREAMBLE  #####
cmake_minimum_required(VERSION 3.24)
project(
    quadcopter
    VERSION 0.1.0
    LANGUAGES C CXX
)


#####  SETTINGS  #####
set_property(
    CACHE CMAKE_BUILD_TYPE
    PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel
)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(
    QUADCOPTER_WARNINGS -Wall -Wextra -Wpedantic
    CACHE STRING "Warning flags when building quadcopter targets"
)
option(BUILD_TESTING "Build tests" OFF)


#####  DEPENDENCIES  #####
find_package(Boost 1.74 REQUIRED)
find_package(spdlog 1.11.0 REQUIRED)
if(CMAKE_SYSTEM_NAME STREQUAL Linux)
    find_package(Gpiod 1.6.2 REQUIRED)
endif()
if(BUILD_TESTING)
    find_package(GTest 1.13.0 REQUIRED)
endif()


#####  MAIN TARGETS  #####
add_subdirectory(src/flight-controller)
if(CMAKE_SYSTEM_NAME STREQUAL Linux)
    # TODO: remove and use mocking for tests
    # add_subdirectory(src/i2c-stub-plain)
endif()


#####  TESTING  #####
if(PROJECT_IS_TOP_LEVEL)
    enable_testing()
endif()
