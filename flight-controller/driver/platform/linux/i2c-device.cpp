#include <cassert>
#include <stdexcept>

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "i2c-device.h"
#include "error.h"

using namespace driver::i2c;

/**
 * @brief Construct a I2C device.
 *
 * @param address LinuxDevice I2C address to communicate with.
 */
LinuxDevice::LinuxDevice(std::uint16_t address)
    : address{address},
      bus_fd{I2C_BUS_PATH, O_RDWR},
      state{State::empty},
      message_buffers{},
      messages{}
{
    unsigned long funcs{};
    util::check_syscall(ioctl(bus_fd, I2C_FUNCS, &funcs));

    // Need I2C transfer functionality
    if ((funcs & I2C_FUNC_I2C) == 0) {
        throw std::runtime_error{"I2C_FUNC_I2C functionality not supported"};
    }

}

void LinuxDevice::read(uint8_t& data)
{
    switch (state) {
    case State::empty:
    case State::writing:
        messages.push_back(
            i2c_msg{
                .addr = address,
                .flags = I2C_M_RD,
                .len = 0,
                .buf = nullptr,
            }
        );
        message_buffers.emplace_back(
            MessageBuffer::Direction::read,
            messages.back());
        break;
    case State::reading:
        break;
    default:
        throw std::logic_error{"state not implemented"};
    }

    message_buffers.back().append(&data);

    state = State::reading;
}

void LinuxDevice::write(uint8_t data)
{
    switch (state) {
    case State::empty:
    case State::reading:
        messages.push_back(
            i2c_msg{
                .addr = address,
                .flags = 0,
                .len = 0,
                .buf = nullptr,
            }
        );
        message_buffers.emplace_back(
            MessageBuffer::Direction::write,
            messages.back());
        break;
    case State::writing:
        break;
    default:
        throw std::logic_error{"state not implemented"};
    }

    message_buffers.back().append(data);

    state = State::writing;
}

void LinuxDevice::transmit()
{
    i2c_rdwr_ioctl_data i2c_data{
        .msgs = messages.data(),
        .nmsgs = messages.size(),
    };

    util::check_syscall(ioctl(bus_fd, I2C_RDWR, &i2c_data));

    for (auto& message_buffer : message_buffers) {
        if (message_buffer.direction != MessageBuffer::Direction::read) {
            continue;
        }
        assert(message_buffer.buffer.size() == message_buffer.out_buffer.size());
        for (std::size_t i = 0; i < message_buffer.buffer.size(); ++i) {
            *message_buffer.out_buffer[i] = message_buffer.buffer[i];
        }
    }

    message_buffers.clear();
    messages.clear();
    state = State::empty;
}

LinuxDevice::MessageBuffer::MessageBuffer(
    LinuxDevice::MessageBuffer::Direction direction, i2c_msg& i2c_message)
    : direction{direction},
      message{&i2c_message},
      buffer{},
      out_buffer{}
{
    // Insert dummy data to guarantee some space has been allocated so
    // buffer.data() will not return nullptr.
    buffer.push_back(0);
    buffer.pop_back();

    message->len = 0;
    message->buf = buffer.data();
}

void LinuxDevice::MessageBuffer::append(std::uint8_t data)
{
    buffer.push_back(data);
    ++message->len;
}

void LinuxDevice::MessageBuffer::append(std::uint8_t* data)
{
    buffer.push_back(0);
    out_buffer.push_back(data);
    ++message->len;
}
