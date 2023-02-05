#include <cassert>

#include <boost/range/combine.hpp>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include "error.h"
#include "i2c-device.h"

using namespace driver::i2c;

/**
 * Construct an I2C device.
 * @param[in] address I2C address of the device to communicate with.
 */
LinuxDevice::LinuxDevice(std::uint16_t address)
    : address{address}, bus_fd{I2C_BUS_PATH, O_RDWR}, state{State::empty}, message_buffers{}
{
    assert(has_required_functionality());
}

/**
 * Read data into argument from I2C device.
 *
 * Start a new transaction if one is not already started. If the transaction
 * state is currently empty or writing, construct a new ReadMessageBuffer and
 * change to the reading state; Otherwise, append to the current
 * ReadMessageBuffer.
 *
 * data will be populated after transmit is called.
 *
 * @param[out] data Reference to populate after transmitting the complete
 *                  transaction.
 */
void LinuxDevice::read(std::uint8_t& data)
{
    switch (state) {
    case State::empty:
    case State::writing:
        message_buffers.emplace_back(ReadMessageBuffer{});
        break;
    case State::reading:
        break;
    default:
        assert(false);// Device transaction state not implemented
    }

    auto& read_buffer = std::get<ReadMessageBuffer>(message_buffers.back());
    read_buffer.buffer.emplace_back(0);
    read_buffer.out_buffer.emplace_back(&data);
    assert(read_buffer.buffer.size() < std::numeric_limits<std::uint16_t>::max());

    state = State::reading;
}

/**
 * Write data to I2C device.
 *
 * Start a new transaction if one is not already started. If the transaction
 * state is currently empty or reading, construct a new WriteMessageBuffer and
 * change to the writing state; Otherwise, append to the current
 * WriteMessageBuffer.
 *
 * data is actually written after transmit is called.
 *
 * @param[in] data Data to write to the I2C device.
 */
void LinuxDevice::write(std::uint8_t data)
{
    switch (state) {
    case State::empty:
    case State::reading:
        message_buffers.emplace_back(WriteMessageBuffer{});
        break;
    case State::writing:
        break;
    default:
        assert(false);// Device transaction state not implemented
    }

    auto& write_buffer = std::get<WriteMessageBuffer>(message_buffers.back());
    write_buffer.buffer.emplace_back(data);
    assert(write_buffer.buffer.size() < std::numeric_limits<std::uint16_t>::max());

    state = State::writing;
}

/**
 * Send complete transaction to I2C device.
 *
 * After transmission, all data read is populated and the device transaction
 * state becomes empty.
 *
 * @post Device transaction state is empty.
 * @post message_buffers is empty.
 */
void LinuxDevice::transmit()
{
    std::vector<i2c_msg> i2c_messages{};
    for (auto& message_buffer : message_buffers) {
        auto* buf = std::visit([](auto&& arg) { return &arg.buffer; }, message_buffer);
        i2c_messages.emplace_back(
            i2c_msg{.addr = address,
                    .flags = static_cast<std::uint16_t>(
                        std::holds_alternative<ReadMessageBuffer>(message_buffer) ? I2C_M_RD : 0),
                    .len = static_cast<std::uint16_t>(buf->size()),
                    .buf = buf->data()});
    }

    i2c_rdwr_ioctl_data i2c_data{
        .msgs = i2c_messages.data(),
        .nmsgs = i2c_messages.size(),
    };

    util::check_syscall(ioctl(bus_fd, I2C_RDWR, &i2c_data));

    for (auto& message_buffer : message_buffers) {
        auto* read_buffer = std::get_if<ReadMessageBuffer>(&message_buffer);
        if (read_buffer == nullptr) { continue; }

        assert(read_buffer->buffer.size() == read_buffer->out_buffer.size());
        for (auto [data, out] : boost::combine(read_buffer->buffer, read_buffer->out_buffer)) {
            *out = data;
        }
    }

    message_buffers.clear();
    state = State::empty;

    assert(message_buffers.empty());
    assert(state == State::empty);
}

/**
 * Check if the device has the required functionality. This is a defensive check
 * because the functionality of the system can be verified ahead of time and
 * does not change.
 * @return true if the system has the required functionality for operation,
 * otherwise false.
 */
bool LinuxDevice::has_required_functionality() const
{
    unsigned long funcs;
    util::check_syscall(ioctl(bus_fd, I2C_FUNCS, &funcs));

    // I2C_FUNC_I2C function is required in order to use I2C_RDWR ioctl function
    return (funcs & I2C_FUNC_I2C) != 0;
}
