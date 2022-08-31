#include <array>
#include <cassert>
#include <cstdint>
#include <string>
#include <stdexcept>
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
        throw std::runtime_error{"I2C_FUNC_I2C functionality not supported"};
    }

    // Need NOSTART functionality to avoid a repeated start during write
    if ((funcs & I2C_FUNC_NOSTART) == 0) {
        throw std::runtime_error{"I2C_FUNC_NOSTART functionality not supported"};
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

std::uint8_t I2CDev::read(std::uint8_t reg) const
{
    std::uint8_t result;
    std::array<i2c_msg, 2> msgs{
            i2c_msg{
                    .addr = address,
                    .flags = 0,
                    .len = 1,
                    .buf = &reg,
            },
            i2c_msg{
                    .addr = address,
                    .flags = I2C_M_RD,
                    .len = 1,
                    .buf = &result,
            }
    };

    i2c_rdwr_ioctl_data data{
            .msgs = msgs.data(),
            .nmsgs = msgs.size(),
    };
    check_syscall(ioctl(bus_fd, I2C_RDWR, &data));

    return result;
}

std::vector<std::uint8_t> I2CDev::read(std::uint8_t start_reg, std::uint16_t length) const
{
    assert(length > 0);  // Cannot read 0 bytes

    std::vector<std::uint8_t> result(length);
    std::array<i2c_msg, 2> msgs{
            i2c_msg{
                    .addr = address,
                    .len = 1,
                    .buf = &start_reg,
            },
            i2c_msg{
                    .addr = address,
                    .flags = I2C_M_RD,
                    .len = length,
                    .buf = result.data(),
            }
    };

    i2c_rdwr_ioctl_data data{
            .msgs = msgs.data(),
            .nmsgs = msgs.size(),
    };
    check_syscall(ioctl(bus_fd, I2C_RDWR, &data));

    return result;
}

void I2CDev::write(std::uint8_t reg, std::uint8_t byte) const
{
    std::array<i2c_msg, 2> msgs = {
            i2c_msg{
                    .addr = address,
                    .len = 1,
                    .buf = &reg
            },
            i2c_msg{
                    .addr = address,
                    .flags = I2C_M_NOSTART,
                    .len = 1,
                    .buf = &byte
            },
    };

    i2c_rdwr_ioctl_data data{
            .msgs = msgs.data(),
            .nmsgs = msgs.size(),
    };
    check_syscall(ioctl(bus_fd, I2C_RDWR, &data));
}

void I2CDev::write(std::uint8_t reg, const std::vector<std::uint8_t>& buffer) const
{
    std::array<i2c_msg, 2> msgs = {
            i2c_msg{
                    .addr = address,
                    .len = 1,
                    .buf = &reg
            },
            i2c_msg{
                    .addr = address,
                    .flags = I2C_M_NOSTART,
                    .len = static_cast<std::uint16_t>(buffer.size() + 1),
                    .buf = const_cast<std::uint8_t*>(buffer.data())
            },
    };

    i2c_rdwr_ioctl_data data{
            .msgs = msgs.data(),
            .nmsgs = msgs.size(),
    };
    check_syscall(ioctl(bus_fd, I2C_RDWR, &data));
}
