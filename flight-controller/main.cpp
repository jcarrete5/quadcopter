#include <iostream>
#include <iomanip>

#include "driver/mpu6050.h"

int main()
{
    MPU6050 mpu{MPU6050::Config{}};
    std::cerr << "WHO_AM_I -> " << std::showbase << std::hex << mpu.who_am_i() << '\n';

    std::cerr << "main thread start sleeping\n";
    std::this_thread::sleep_for(std::chrono::seconds{11});
    std::cerr << "main thread done sleeping\n";
    return 0;
}
