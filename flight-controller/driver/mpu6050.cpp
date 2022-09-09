#include <cmath>
#include <cstdint>
#include <iostream>

#include <gpiod.hpp>
#include <iomanip>

#include "mpu6050.h"
#include "utils/defer.h"
#include "utils/error.h"

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
      event_thread{[this, &config]() { event_loop(config); }}
{
    if (who_am_i() != MPU6050::i2c_address) {
        throw std::runtime_error{"I2C device is not an MPU6050"};
    }

    check_pthread_call(pthread_setname_np(event_thread.native_handle(), "MPU6050_event"));
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
    Defer cleanup_line{[&line]() { line.release(); }};
    line.request(
        {"flight-controller",  // TODO probably replace with some constant name of the executable
         gpiod::line_request::EVENT_RISING_EDGE,
         gpiod::line_request::FLAG_BIAS_DISABLE
        }
    );

    while (!event_loop_interrupted) {
        const bool timed_out{!line.event_wait(config.event_wait_timeout)};
        if (timed_out) {
            continue;
        }

        gpiod::line_event ev{line.event_read()};
        std::cerr << std::dec << "Event detected: [" << duration_cast<microseconds>(ev.timestamp).count()
                  << "] " << ev.event_type << '\n';
        // TODO Request sensor data from I2C bus, buffer it, and clear interrupt status
    }
}
