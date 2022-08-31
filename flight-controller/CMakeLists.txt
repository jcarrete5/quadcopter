project(flight-controller
        VERSION 0.1.0
        LANGUAGES CXX)

find_package(spdlog 1.8.1 REQUIRED)

add_executable(flight-control "main.cpp" "driver/i2c-dev.cpp" "driver/mpu6050.cpp")
target_link_libraries(flight-control spdlog::spdlog)
