target_sources(
    driver
    PRIVATE i2c-device.cpp
)
target_compile_definitions(
    driver
    PRIVATE I2C_BUS_PATH="/dev/i2c-1"
)
target_link_libraries(
    driver
    PRIVATE Gpiod::gpiod
)
