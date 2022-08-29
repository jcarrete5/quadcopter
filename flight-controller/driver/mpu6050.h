#ifndef MPU6050_H
#define MPU6050_H

#include <cstdint>
#include <sstream>
#include <string>

#include <unistd.h>

#include "i2c-dev.h"

class MPU6050 {
public:
    explicit MPU6050(const std::string& i2c_bus, bool ad0_low = true);

    bool self_test();
    int who_am_i();

private:
    I2CDev device;

    /// Register address values.
    enum class Register {
        self_test_x = 13,
        self_test_y,
        self_test_z,
        self_test_a,
        smplrt_div = 25,
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
};

#endif /* !defined MPU6050_H */
