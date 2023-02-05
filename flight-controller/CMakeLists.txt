project(flight-controller
    VERSION 0.1.0
    LANGUAGES CXX)

find_package(spdlog 1.8.1 REQUIRED)
find_package(Boost 1.74.0 REQUIRED)

if ("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL Linux)
    find_package(PkgConfig 0.29.2 REQUIRED)
    pkg_check_modules(gpiod REQUIRED libgpiodcxx>=1.6.2 IMPORTED_TARGET)
endif ()

add_library(util
    util/defer.cpp
    "$<$<PLATFORM_ID:Linux,Darwin>:util/platform/posix/file-descriptor.cpp>")
target_include_directories(util
    PUBLIC util/public/
    PRIVATE util/)

add_library(driver
    "$<$<PLATFORM_ID:Linux>:driver/platform/linux/i2c-device.cpp>")
target_include_directories(driver
    PUBLIC driver/public
    PRIVATE driver/)
target_link_libraries(driver
    PRIVATE "$<$<PLATFORM_ID:Linux>:PkgConfig::gpiod>"
    PRIVATE util)
target_compile_definitions(driver
    PRIVATE "$<$<PLATFORM_ID:Linux>:I2C_BUS_PATH=\"/dev/i2c-1\">")

add_executable(flight-control
    main.cpp)
target_link_libraries(flight-control
    PRIVATE spdlog::spdlog
    PRIVATE Boost::boost
    PRIVATE util
    PRIVATE driver)

if (BUILD_TESTING)
    add_subdirectory(tests)
endif()
