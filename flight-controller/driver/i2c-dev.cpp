#include <array>
#include <cassert>
#include <cstdint>
#include <limits>
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

/**
 * @brief Construct a I2C device.
 *
 * @param i2c_bus Path to an I2C device file.
 * @param address Device I2C address to communicate with.
 */
I2CDev::I2CDev(const std::string& i2c_bus, std::uint16_t address)
    : bus_fd{open(i2c_bus.c_str(), O_RDWR)}, address{address}
{
    check_syscall(bus_fd);

    unsigned long funcs{};
    check_syscall(ioctl(bus_fd, I2C_FUNCS, &funcs));

    // Need I2C transfer functionality
    if ((funcs & I2C_FUNC_I2C) == 0) {
        throw std::runtime_error{"I2C_FUNC_I2C functionality not supported"};
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

/**
 * @brief Read a byte from a device register.
 *
 * @param reg Device register to read from.
 * @return The byte read from the device register.
 */
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

/**
 * @brief Read consecutive bytes starting from a register.
 *
 * This function will fail when trying to read 0 bytes.
 *
 * @param start_reg Device register to start reading bytes from.
 * @param length Number of consecutive bytes to read.
 * @return A vector containing the bytes read.
 */
std::vector<std::uint8_t> I2CDev::read(std::uint8_t start_reg, std::uint16_t length) const
{
    assert(length > 0);  // Cannot read 0 bytes

    std::vector<std::uint8_t> result(length);
    std::array<i2c_msg, 2> msgs{
        i2c_msg{
            .addr = address,
            .flags = 0,
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

/**
 * @brief Write a byte to a device register.
 *
 * @param reg Device register to write a byte into.
 * @param byte Byte to write into the device register.
 */
void I2CDev::write(std::uint8_t reg, std::uint8_t byte) const
{
    std::array<std::uint8_t, 2> buffer = {reg, byte};
    std::array<i2c_msg, 2> msgs = {
        i2c_msg{
            .addr = address,
            .flags = 0,
            .len = buffer.size(),
            .buf = buffer.data()
        },
    };

    i2c_rdwr_ioctl_data data{
        .msgs = msgs.data(),
        .nmsgs = msgs.size(),
    };
    check_syscall(ioctl(bus_fd, I2C_RDWR, &data));
}

/**
 * @brief Write a buffer of bytes to consecutive registers.
 *
 * The buffer must not be empty and it's size must fit within a std::uint16_t.
 *
 * Each byte from buffer is written to consecutive registers on the device
 * starting with the specified register.
 *
 * Example:
 * write(24, std::vector<std::uint8_t>{1, 2, 3, 4})
 *
 * Register 24 => 1
 * Register 25 => 2
 * Register 26 => 3
 * Register 27 => 4
 *
 * @param reg Device register to start writing data into.
 * @param buffer Buffer containing bytes to write.
 */
void I2CDev::write(std::uint8_t reg, std::vector<std::uint8_t> buffer) const
{
    // buffer must not be empty
    assert(!buffer.empty());

    // buffer must not exceed std::uint16_t maximum to be used as a length in an i2c_msg
    buffer.insert(buffer.cbegin(), reg);
    assert(buffer.size() <= std::numeric_limits<std::uint16_t>::max());

    std::array<i2c_msg, 2> msgs = {
        i2c_msg{
            .addr = address,
            .flags = 0,
            .len = static_cast<std::uint16_t>(buffer.size()),
            .buf = buffer.data()
        },
    };

    i2c_rdwr_ioctl_data data{
        .msgs = msgs.data(),
        .nmsgs = msgs.size(),
    };
    check_syscall(ioctl(bus_fd, I2C_RDWR, &data));
}
