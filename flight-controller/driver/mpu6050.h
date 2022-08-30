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
};



#endif /* !defined MPU6050_H */
