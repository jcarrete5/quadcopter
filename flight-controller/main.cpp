#include <iostream>
#include <iomanip>

#include "driver/mpu6050.h"

int main()
{
    MPU6050 mpu{"/dev/i2c-1"};
    std::cout << "WHO_AM_I -> " << std::showbase << std::setbase(16) << mpu.who_am_i() << '\n';
    return 0;
}
