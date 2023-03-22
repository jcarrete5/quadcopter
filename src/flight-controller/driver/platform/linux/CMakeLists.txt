find_package(PkgConfig 0.29.2 REQUIRED)
pkg_check_modules(gpiod REQUIRED libgpiodcxx>=1.6.2 IMPORTED_TARGET)

target_sources(
    driver
    PRIVATE
        i2c-device.cpp
)
target_compile_definitions(
    driver
    PRIVATE
        I2C_BUS_PATH="/dev/i2c-1"
)
target_link_libraries(
    driver
    PRIVATE
        PkgConfig::gpiod
)
