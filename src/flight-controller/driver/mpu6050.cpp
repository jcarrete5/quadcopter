#include <cmath>
#include <cstdint>
#include <iostream>
#include <mutex>

#include <gpiod.hpp>
#include <iomanip>

#include "mpu6050.h"
#include "defer.h"
#include "error.h"

#define XA_OFFS_USRH 6

/**
 * @brief MPU60X0 register names mappings.
 *
 * See MPU-6000-Register-Map1.pdf and 'MPU HW Offset Registers 1.2.pdf'
 * in Google Drive for more information about the registers.
 */
enum class [[maybe_unused]] Register : std::uint8_t {
    xa_offs_usrh = 6,
    xa_offs_usrl,
    ya_offs_usrh,
    ya_offs_usrl,
    za_offs_usrh,
    za_offs_usrl,
    self_test_x = 13,
    self_test_y,
    self_test_z,
    self_test_a,
    xg_offs_usrh = 19,
    xg_offs_usrl,
    yg_offs_usrh,
    yg_offs_usrl,
    zg_offs_usrh,
    zg_offs_usrl,
    smprt_div = 25,
    config,
    gyro_config,
    accel_config,
    fifo_en = 35,
    i2c_mst_ctrl,
    i2c_slv0_addr,
    i2c_slv0_reg,
    i2c_slv0_ctrl,
    i2c_slv1_addr,
    i2c_slv1_reg,
    i2c_slv1_ctrl,
    i2c_slv2_addr,
    i2c_slv2_reg,
    i2c_slv2_ctrl,
    i2c_slv3_addr,
    i2c_slv3_reg,
    i2c_slv3_ctrl,
    i2c_slv4_addr,
    i2c_slv4_reg,
    i2c_slv4_do,
    i2c_slv4_ctrl,
    i2c_slv4_di,
    i2c_mst_status,
    int_pin_cfg,
    int_enable,
    int_status = 58,
    accel_xout_h,
    accel_xout_l,
    accel_yout_h,
    accel_yout_l,
    accel_zout_h,
    accel_zout_l,
    temp_out_h,
    temp_out_l,
    gyro_xout_h,
    gyro_xout_l,
    gyro_yout_h,
    gyro_yout_l,
    gyro_zout_h,
    gyro_zout_l,
    ext_sens_data_00,
    ext_sens_data_01,
    ext_sens_data_02,
    ext_sens_data_03,
    ext_sens_data_04,
    ext_sens_data_05,
    ext_sens_data_06,
    ext_sens_data_07,
    ext_sens_data_08,
    ext_sens_data_09,
    ext_sens_data_10,
    ext_sens_data_11,
    ext_sens_data_12,
    ext_sens_data_13,
    ext_sens_data_14,
    ext_sens_data_15,
    ext_sens_data_16,
    ext_sens_data_17,
    ext_sens_data_18,
    ext_sens_data_19,
    ext_sens_data_20,
    ext_sens_data_21,
    ext_sens_data_22,
    ext_sens_data_23,
    i2c_slv0_do = 99,
    i2c_slv1_do,
    i2c_slv2_do,
    i2c_slv3_do,
    i2c_mst_delay_ct,
    signal_path_reset,
    user_ctrl = 106,
    pwr_mgmt_1,
    pwr_mgmt_2,
    fifo_counth = 114,
    fifo_countl,
    fifo_r_w,
    who_am_i,
};

MPU6050::MPU6050(const Config& config)
    : device{config.bus_path, config.i2c_address},
      event_loop_interrupted{false},
      event_thread{[this, config]() { event_loop(config); }},
      data_buffer_mutex{},
      data_in_buffer{},
      data_buffer{config.data_buffer_capacity}
{
    if (who_am_i() != MPU6050::i2c_address) {
        throw std::runtime_error{"I2C device is not an MPU6050"};
    }

    util::check_pthread_call(pthread_setname_np(event_thread.native_handle(), "MPU6050_event"));

    reset();

    // Configure full-scale ranges
    device.write(static_cast<std::uint8_t>(Register::gyro_config), static_cast<std::uint8_t>(config.gyro_range) << 3);
    device.write(static_cast<std::uint8_t>(Register::accel_config), static_cast<std::uint8_t>(config.accel_range) << 3);

    // Enable DATA_READY interrupt
    device.write(static_cast<std::uint8_t>(Register::int_enable), 0x01);

    // Wake up device
    device.write(static_cast<std::uint8_t>(Register::pwr_mgmt_1), 0x00);
}

MPU6050::~MPU6050()
{
    event_loop_interrupted = true;
    try {
        std::cerr << "MPU6050_event joining\n";
        event_thread.join();
        std::cerr << "MPU6050_event joined\n";
    } catch (std::exception& e) {
        std::cerr << "error joining MPU6050::event_thread: " << e.what() << '\n';
    } catch (...) {
        std::cerr << "error joining MPU6050::event_thread\n";
    }
}

/**
 * @brief Perform a self-test on all axes.
 *
 * The self-test verifies the MPU is operating within the limits of its
 * specification.
 *
 * @returns True if the self-test passes, otherwise false.
 */
bool MPU6050::self_test() const
{
    std::vector<std::uint8_t> data{
        device.read(static_cast<std::uint8_t>(Register::self_test_x), 4)
    };
    std::uint8_t r0{data[0]};  // SELF_TEST_X
    std::uint8_t r1{data[1]};  // SELF_TEST_Y
    std::uint8_t r2{data[2]};  // SELF_TEST_Z
    std::uint8_t r3{data[3]};  // SELF_TEST_A

    auto xg_test{static_cast<std::uint8_t>(r0 & 0x1f)};
    auto yg_test{static_cast<std::uint8_t>(r1 & 0x1f)};
    auto zg_test{static_cast<std::uint8_t>(r2 & 0x1f)};
    auto xa_test{static_cast<std::uint8_t>((r0 & 0xe0) >> 3 | (r3 & 0x30) >> 4)};
    auto ya_test{static_cast<std::uint8_t>((r1 & 0xe0) >> 3 | (r3 & 0x0c) >> 2)};
    auto za_test{static_cast<std::uint8_t>((r2 & 0xe0) >> 3 | (r3 & 0x03) >> 0)};

    double ft_xg{xg_test == 0 ? 0 : 25 * 131 * std::pow(1.046, xg_test - 1)};
    double ft_yg{yg_test == 0 ? 0 : -25 * 131 * std::pow(1.046, yg_test - 1)};
    double ft_zg{zg_test == 0 ? 0 : 25 * 131 * std::pow(1.046, zg_test - 1)};
    double ft_xa{xa_test == 0 ? 0 : 4096 * 0.34 * std::pow(0.92 / 0.34, (xa_test - 1) / 30)};
    double ft_ya{ya_test == 0 ? 0 : 4096 * 0.34 * std::pow(0.92 / 0.34, (ya_test - 1) / 30)};
    double ft_za{za_test == 0 ? 0 : 4096 * 0.34 * std::pow(0.92 / 0.34, (za_test - 1) / 30)};

    device.write(static_cast<std::uint8_t>(Register::gyro_config), static_cast<std::uint8_t>(0b11100000));
    device.write(static_cast<std::uint8_t>(Register::accel_config), static_cast<std::uint8_t>(0xf0));

    return true;
}

/**
 * @brief Read the contents of the WHO_AM_I register.
 *
 * @returns The data contained within the WHO_AM_I register on the MPU.
 */
int MPU6050::who_am_i() const
{
    return device.read(static_cast<std::uint8_t>(Register::who_am_i));
}

void MPU6050::event_loop(const Config& config)
{
    using namespace std::chrono;

    gpiod::chip chip{config.gpio_path, gpiod::chip::OPEN_BY_PATH};
    gpiod::line line{chip.get_line(config.interrupt_line)};
    util::Defer cleanup_line{[&line]() { line.release(); }};
    line.request(
        {"flight-controller",  // TODO probably replace with some constant name of the executable
         gpiod::line_request::EVENT_RISING_EDGE,
         gpiod::line_request::FLAG_BIAS_DISABLE
        }
    );

    while (!event_loop_interrupted) {
        // TODO: Maybe one day this can just be indefinite and the thread can be interrupted properly
        const bool timed_out{!line.event_wait(config.event_wait_timeout)};
        if (timed_out) {
            continue;
        }

        gpiod::line_event event{line.event_read()};
        std::cerr << std::dec << "Event detected: [" << event.timestamp.count() << "] " << event.event_type << '\n';

        try {
            // Read and clear interrupt status on MPU
            std::uint8_t interrupt_status{device.read(static_cast<std::uint8_t>(Register::int_status))};
            if (interrupt_status != 0x01) {
                std::cerr << "Skipping non-data-ready interrupt: " << std::hex << std::showbase
                          << static_cast<int>(interrupt_status) << '\n';
                std::cerr << std::dec;
                continue;
            }

            // Save sensor data
            std::vector<std::uint8_t> raw_data = device.read(static_cast<std::uint8_t>(Register::accel_xout_h), 14);
            push_sample(
                {
                    event.timestamp,
                    static_cast<std::int16_t>(raw_data[0] << 8 | raw_data[1])
                        * gyro_full_scale_factor(config.gyro_range),
                    static_cast<std::int16_t>(raw_data[2] << 8 | raw_data[3])
                        * gyro_full_scale_factor(config.gyro_range),
                    static_cast<std::int16_t>(raw_data[4] << 8 | raw_data[5])
                        * gyro_full_scale_factor(config.gyro_range),
                    static_cast<std::int16_t>(raw_data[6] << 8 | raw_data[7]) / 340.0 + 36.53,
                    static_cast<std::int16_t>(raw_data[8] << 8 | raw_data[9])
                        * accel_full_scale_factor(config.accel_range),
                    static_cast<std::int16_t>(raw_data[10] << 8 | raw_data[11])
                        * accel_full_scale_factor(config.accel_range),
                    static_cast<std::int16_t>(raw_data[12] << 8 | raw_data[13])
                        * accel_full_scale_factor(config.accel_range),
                }
            );
        } catch (std::system_error& e) {
            if (e.code().value() == 121) {
                std::cerr << "Remote I/O error during MPU6050 event loop. Skipping failed read\n";
            } else {
                throw;
            }
        }
    }
}

/**
 * @brief Reset device registers and signal paths.
 */
void MPU6050::reset() const
{
    device.write(static_cast<std::uint8_t>(Register::pwr_mgmt_1), 0x80);
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
    device.write(static_cast<std::uint8_t>(Register::signal_path_reset), 0x07);
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
}

/**
 * @brief Push new sample data at the back of the buffer.
 *
 * The buffer mutex is locked before attempted to add a new sample. As a result,
 * this function may block briefly if the lock cannot be acquired immediately.
 *
 * @param sample New sample to add.
 */
void MPU6050::push_sample(const MPU6050::Sample& sample)
{
    {
        std::lock_guard buffer_guard{data_buffer_mutex};
        data_buffer.push_back(sample);
    }
    data_in_buffer.notify_all();
}

/**
 * @brief Remove the sample at the front of the buffer.
 *
 * This function will block if the buffer is empty or if the buffer mutex
 * cannot be locked immediately.
 *
 * @return The removed Sample.
 */
MPU6050::Sample MPU6050::pop_sample()
{
    std::unique_lock buffer_guard{data_buffer_mutex};

    // Wait until buffer has data
    data_in_buffer.wait(buffer_guard, [this]() { return !data_buffer.empty(); });

    Sample sample{data_buffer[0]};
    data_buffer.pop_front();
    return sample;
}

std::ostream& operator<<(std::ostream& os, const MPU6050::Sample& sample)
{
    os << '[' << sample.timestamp.count() << "] Accelerometer: (" << sample.accel_x << ", " << sample.accel_y << ", "
       << sample.accel_z << ") Gyroscope: (" << sample.gyro_x << ", " << sample.gyro_y << ", " << sample.gyro_z
       << ") Temperature: " << sample.temperature;
    return os;
}

/**
 * @param range The gyroscope full-scale range.
 * @return The conversion factor used to convert raw gyroscope data to degrees per second.
 */
double MPU6050::gyro_full_scale_factor(MPU6050::GyroRange range)
{
    switch (range) {
    case GyroRange::_250_degrees_per_second:
        return 1.0 / 131.0;
    case GyroRange::_500_degrees_per_second:
        return 1.0 / 65.5;
    case GyroRange::_1000_degrees_per_second:
        return 1.0 / 32.8;
    case GyroRange::_2000_degrees_per_second:
        return 1.0 / 16.4;
    default:
        // Enumerator not implemented
        assert(false);
    }
}

/**
 * @param range The accelerometer full-scale range.
 * @return The conversion factor used to convert raw accelerometer data to g-force.
 */
double MPU6050::accel_full_scale_factor(MPU6050::AccelRange range)
{
    switch (range) {
    case AccelRange::_2g:
        return 1.0 / 16384.0;
    case AccelRange::_4g:
        return 1.0 / 8192.0;
    case AccelRange::_8g:
        return 1.0 / 4096.0;
    case AccelRange::_16g:
        return 1.0 / 2048.0;
    default:
        // Enumerator not implemented
        assert(false);
    }
}
