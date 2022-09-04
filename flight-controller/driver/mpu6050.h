#ifndef MPU6050_H
#define MPU6050_H

#include <string>

#include "i2c-dev.h"

class MPU6050 {
public:
    /// MPU6050 I2C address when AD0 is driven LOW.
    [[maybe_unused]] static constexpr std::uint16_t i2c_address{0x68};
    /// MPU6050 I2C address when AD0 is driven HIGH.
    [[maybe_unused]] static constexpr std::uint16_t i2c_address_alt{0x69};

    struct Config {
        std::string bus_path = "/dev/i2c-1";
        std::uint16_t i2c_address = MPU6050::i2c_address;
    };

    explicit MPU6050(const Config& cfg);
    MPU6050(const MPU6050& other) = delete;
    MPU6050& operator=(const MPU6050& other) = delete;
    MPU6050(MPU6050&& other) = default;
    MPU6050& operator=(MPU6050&& other) = default;

    bool self_test() const;
    [[nodiscard]] int who_am_i() const;

private:
    I2CDev device;
};

#endif /* !defined MPU6050_H */
