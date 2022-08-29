#include <cstdint>
#include <string>

#include "mpu6050.h"

MPU6050::MPU6050(const std::string& i2c_bus, bool ad0_low)
        : device{i2c_bus, static_cast<std::uint16_t>(ad0_low ? 0x68 : 0x69)}
{
}

/**
 * @brief Perform a self-test on all axes.
 *
 * The self-test verifies the MPU is operating within the limits of its
 * specification.
 *
 * @returns True if the self-test passes, otherwise false.
 */
//bool MPU6050::self_test()
//{
//    uint8_t r0 = read_register<uint8_t>(Register::self_test_x);
//    uint8_t r1 = read_register<uint8_t>(Register::self_test_y);
//    uint8_t r2 = read_register<uint8_t>(Register::self_test_z);
//    uint8_t r3 = read_register<uint8_t>(Register::self_test_a);
//
//    uint8_t xg_test = r0 & 0x1f;
//    uint8_t yg_test = r1 & 0x1f;
//    uint8_t zg_test = r2 & 0x1f;
//    uint8_t xa_test = (r0 & 0xe0) >> 3 | (r3 & 0x30) >> 4;
//    uint8_t ya_test = (r1 & 0xe0) >> 3 | (r3 & 0x0c) >> 2;
//    uint8_t za_test = (r2 & 0xe0) >> 3 | (r3 & 0x03) >> 0;
//
//    double ft_xg = xg_test == 0 ? 0 : 25 * 131 * std::pow(1.046, xg_test - 1);
//    double ft_yg = yg_test == 0 ? 0 : -25 * 131 * std::pow(1.046, yg_test - 1);
//    double ft_zg = zg_test == 0 ? 0 : 25 * 131 * std::pow(1.046, zg_test - 1);
//    double ft_xa = xa_test == 0 ? 0 : 4096 * 0.34 * std::pow(0.92 / 0.34, (xa_test - 1) / 30);
//    double ft_ya = ya_test == 0 ? 0 : 4096 * 0.34 * std::pow(0.92 / 0.34, (ya_test - 1) / 30);
//    double ft_za = za_test == 0 ? 0 : 4096 * 0.34 * std::pow(0.92 / 0.34, (za_test - 1) / 30);
//
//    write_register(Register::gyro_config, static_cast<uint8_t>(0b11100000));
//    write_register(Register::accel_config, static_cast<uint8_t>(0xf0));
//
//    return true;
//}

/**
 * @brief Read the contents of the WHO_AM_I register.
 *
 * @returns The data contained within the WHO_AM_I register on the MPU.
 */
int MPU6050::who_am_i()
{
    std::vector<uint8_t> data = device.read(static_cast<std::uint8_t>(Register::who_am_i));
    return data[0];
}
