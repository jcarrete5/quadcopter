add_library(driver)
target_sources(
    driver
    PRIVATE i2c-device.cpp
)
target_include_directories(
    driver
    PUBLIC public
    PRIVATE .
)
target_link_libraries(
    driver
    PRIVATE util
)

# TODO: Consider conditionally including this when we want to simulate
add_subdirectory(i2c-stub-plain)

if(CMAKE_SYSTEM_NAME STREQUAL Linux)
    add_subdirectory(platform/linux)
endif()

if(BUILD_TESTING)
    add_subdirectory(test)
endif()
