project(flight-controller
        VERSION 0.1.0
        LANGUAGES CXX)

find_package(spdlog 1.8.1 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(kalman-filter)

add_executable(flight-control "main.cpp" "driver/i2c-dev.cpp" "driver/mpu6050.cpp")
target_link_libraries(flight-control spdlog::spdlog)
target_link_libraries(flight-control Eigen3::Eigen)
