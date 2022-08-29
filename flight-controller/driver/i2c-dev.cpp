#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include "i2c-dev.h"
#include "../error.h"

I2CDev::I2CDev(const std::string& i2c_bus, uint16_t address)
        : bus_fd{open(i2c_bus.c_str(), O_RDWR)}, address{address}
{
    check_syscall(bus_fd);

    unsigned long funcs{};
    check_syscall(ioctl(bus_fd, I2C_FUNCS, &funcs));

    // Need I2C transfer functionality
    if ((funcs & I2C_FUNC_I2C) == 0) {
        throw I2CFunctionNotSupported{"I2C_FUNC_I2C"};
    }
}

I2CDev::I2CDev(const I2CDev& other)
        : bus_fd{dup(other.bus_fd)}, address{other.address}
{
    check_syscall(bus_fd);
}

I2CDev& I2CDev::operator=(I2CDev other)
{
    swap(*this, other);
    return *this;
}

I2CDev::~I2CDev()
{
    close(bus_fd);
    // TODO Log if close fails
}

std::vector<std::uint8_t> I2CDev::read(std::uint8_t start_reg, std::uint16_t length) const
{
    assert(length > 0);  // Cannot read 0 bytes

    std::vector<std::uint8_t> result(length);
    i2c_msg msgs[] = {
            {
                    .addr = address,
                    .flags = 0,
                    .len = 1,
                    .buf = &start_reg,
            },
            {
                    .addr = address,
                    .flags = I2C_M_RD,
                    .len = length,
                    .buf = result.data(),
            }
    };

    i2c_rdwr_ioctl_data data{
            .msgs = msgs,
            .nmsgs = 2,
    };

    check_syscall(ioctl(bus_fd, I2C_RDWR, &data));

    return result;
}
