#ifndef MPU6050_H
#define MPU6050_H

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "i2c-dev.h"

class MPU6050 {
public:
    /// MPU6050 I2C address when AD0 is driven LOW.
    static constexpr std::uint16_t i2c_address{0x68};
    /// MPU6050 I2C address when AD0 is driven HIGH.
    [[maybe_unused]] static constexpr std::uint16_t i2c_address_alt{0x69};

    struct Config {
        std::string bus_path{"/dev/i2c-1"};
        std::uint16_t i2c_address{MPU6050::i2c_address};
        std::string gpio_path{"/dev/gpiochip0"};
        unsigned int interrupt_line{4};
        std::chrono::seconds event_wait_timeout{5};
    };

    explicit MPU6050(const Config& cfg);
    ~MPU6050();
    MPU6050(const MPU6050& other) = delete;
    MPU6050& operator=(const MPU6050& other) = delete;
    MPU6050(MPU6050&& other) = delete;
    MPU6050& operator=(MPU6050&& other) = delete;

    bool self_test() const;
    [[nodiscard]] int who_am_i() const;

private:
    I2CDev device;
    std::atomic<bool> event_loop_interrupted;
    std::thread event_thread;

    void event_loop(const Config& config);
};

#endif /* !defined MPU6050_H */
