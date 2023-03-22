#include <cstdint>
#include <memory>

#include "i2c-device.h"

#ifdef __linux__
#include "platform/linux/i2c-device.h"
using DeviceImpl = driver::i2c::LinuxDevice;
#else
#error "I2C device driver not implemented for the target platform"
#endif

using namespace driver::i2c;

std::unique_ptr<Device> driver::i2c::create_device(std::uint16_t address)
{
    return std::make_unique<DeviceImpl>(address);
}
