project(flight-controller
        VERSION 0.1.0
        LANGUAGES CXX)

find_package(PkgConfig 0.29.2 REQUIRED)
find_package(spdlog 1.8.1 REQUIRED)
pkg_check_modules(gpiod REQUIRED libgpiodcxx>=1.6.2 IMPORTED_TARGET)
find_package(Boost 1.74.0 REQUIRED)

add_executable(flight-control
    main.cpp
    utils/file-descriptor.cpp
    driver/i2c-dev.cpp
    driver/mpu6050.cpp
)
target_include_directories(flight-control PRIVATE ".")
target_link_libraries(flight-control spdlog::spdlog)
target_link_libraries(flight-control PkgConfig::gpiod)
target_link_libraries(flight-control Boost::boost)

if (BUILD_TESTING)
    add_subdirectory(tests)
endif()
