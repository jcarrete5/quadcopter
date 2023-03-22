#include <iostream>

#include "i2c-device.h"

int main()
{
    using namespace driver::i2c;
    std::unique_ptr<Device> dev = create_device(12);
    return 0;
}
