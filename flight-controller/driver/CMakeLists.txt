add_library(driver
    i2c-device.cpp)
target_include_directories(driver
    PUBLIC public
    PRIVATE .)
target_link_libraries(driver
    PRIVATE util)

if(CMAKE_SYSTEM_NAME STREQUAL Linux)
    add_subdirectory(platform/linux)
endif()
