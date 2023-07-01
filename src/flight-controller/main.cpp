#include <iostream>

#include "i2c-device.h"

int main()
{
    using namespace driver;
    auto dev = i2c::create_device(12);
    return 0;
}
