#ifndef MPU6050_H
#define MPU6050_H

#include <atomic>
#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include <boost/circular_buffer.hpp>

#include "i2c-dev.h"


/**
 * @brief MPU6050 device abstraction.
 */
class MPU6050 {
public:
    /// MPU6050 I2C address when AD0 is driven LOW.
    static constexpr std::uint16_t i2c_address{0x68};
    /// MPU6050 I2C address when AD0 is driven HIGH.
    [[maybe_unused]] static constexpr std::uint16_t i2c_address_alt{0x69};

    /// Gyroscope full-scale range.
    enum class [[maybe_unused]] GyroRange : std::uint8_t {
        _250_degrees_per_second,
        _500_degrees_per_second,
        _1000_degrees_per_second,
        _2000_degrees_per_second,
    };

    /// Accelerometer full-scale range.
    enum class [[maybe_unused]] AccelRange : std::uint8_t {
        _2g,
        _4g,
        _8g,
        _16g,
    };

    struct Sample {
        std::chrono::nanoseconds timestamp;
        double accel_x;
        double accel_y;
        double accel_z;
        double temperature;
        double gyro_x;
        double gyro_y;
        double gyro_z;

        friend std::ostream& operator<<(std::ostream& os, const Sample& sample);
    };

    struct Config {
        std::string bus_path{"/dev/i2c-1"};
        std::uint16_t i2c_address{MPU6050::i2c_address};

        std::string gpio_path{"/dev/gpiochip0"};
        /// Raspberry Pi GPIO pin to use for receiving interrupts.
        unsigned int interrupt_line{4};
        std::chrono::seconds event_wait_timeout{5};
        std::size_t data_buffer_capacity{1024};

        GyroRange gyro_range{GyroRange::_250_degrees_per_second};
        AccelRange accel_range{AccelRange::_2g};
    };

    explicit MPU6050(const Config& cfg);
    ~MPU6050();
    MPU6050(const MPU6050& other) = delete;
    MPU6050& operator=(const MPU6050& other) = delete;
    MPU6050(MPU6050&& other) = delete;
    MPU6050& operator=(MPU6050&& other) = delete;

    bool self_test() const;
    [[nodiscard]] int who_am_i() const;
    Sample pop_sample();

private:
    I2CDev device;
    std::atomic<bool> event_loop_interrupted;
    std::thread event_thread;

    std::mutex data_buffer_mutex;
    std::condition_variable data_in_buffer;
    boost::circular_buffer<Sample> data_buffer;

    [[nodiscard]] static double gyro_full_scale_factor(GyroRange range);
    [[nodiscard]] static double accel_full_scale_factor(AccelRange range);
    void event_loop(const Config& config);
    void reset() const;
    void push_sample(const Sample& sample);
};

#endif /* !defined MPU6050_H */
