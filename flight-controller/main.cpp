#include <iostream>

#include "driver/mpu6050.h"

int main()
{
    MPU6050 mpu{MPU6050::Config{}};
    for (int i = 0; i < 1000; ++i) {
        std::cout << mpu.pop_sample() << '\n';
    }
    return 0;
}
