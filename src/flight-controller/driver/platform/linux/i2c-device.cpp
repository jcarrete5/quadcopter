#include <cassert>
#include <numeric>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include "error.h"
#include "i2c-device.h"

using namespace driver::i2c;

Device::MessageTransmitter::MessageTransmitter() : bus_fd{I2C_BUS_PATH, O_RDWR}
{
    assert(has_required_functionality());
}

void Device::MessageTransmitter::transmit(Device& device)
{
    std::vector<i2c_msg> i2c_messages{};
    for (auto& message_buffer : device.message_buffers) {
        assert(i2c_messages.size() != std::numeric_limits<std::uint32_t>::max());
        auto* buf = std::visit([](auto&& arg) { return &arg.buffer; }, message_buffer);
        i2c_messages.emplace_back(
            i2c_msg{.addr = device.address,
                    .flags = static_cast<std::uint16_t>(
                        std::holds_alternative<ReadMessageBuffer>(message_buffer) ? I2C_M_RD : 0),
                    .len = static_cast<std::uint16_t>(buf->size()),
                    .buf = buf->data()});
    }

    i2c_rdwr_ioctl_data i2c_data{
        .msgs = i2c_messages.data(),
        .nmsgs = static_cast<std::uint32_t>(i2c_messages.size()),
    };

    util::check_syscall(ioctl(bus_fd, I2C_RDWR, &i2c_data));
}

/**
 * Check if the device has the required functionality. This is a defensive check
 * because the functionality of the system can be verified ahead of time and
 * does not change.
 * @return true if the system has the required functionality for operation,
 * otherwise false.
 */
bool Device::MessageTransmitter::has_required_functionality() const
{
    unsigned long funcs;
    util::check_syscall(ioctl(bus_fd, I2C_FUNCS, &funcs));

    // I2C_FUNC_I2C function is required in order to use I2C_RDWR ioctl function
    return (funcs & I2C_FUNC_I2C) != 0;
}
