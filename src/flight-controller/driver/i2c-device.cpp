#include <boost/range/combine.hpp>
#include <cassert>
#include <cstdint>
#include <memory>

#include "i2c-device.h"

#ifdef __linux__
#include "platform/linux/i2c-device.h"
#else
#error "I2C device driver not implemented for the target platform"
#endif

using namespace driver::i2c;

/**
 * Construct an I2C device.
 *
 * @param[in] address I2C address of the device to communicate with.
 */
Device::Device(std::uint16_t address_)
    : transmitter{std::make_unique<MessageTransmitter>()}, address{address_}, state{State::empty},
      message_buffers{}
{}

Device::~Device() = default;

Device driver::i2c::create_device(std::uint16_t address)
{
    return Device{address};
}

/**
 * Read data into argument from I2C device.
 *
 * Starts a new transaction if one is not already started. data will be
 * populated after transmit is called.
 *
 * @param[out] data Reference to populate after transmitting the complete
 *                  transaction.
 */
void Device::read(std::uint8_t& data)
{
    switch (state) {
    case State::empty:
    case State::writing:
        message_buffers.emplace_back(ReadMessageBuffer{});
        break;
    case State::reading:
        break;
    default:
        assert(!"device transaction state not implemented");
    }

    auto& read_buffer = std::get<ReadMessageBuffer>(message_buffers.back());
    assert(read_buffer.buffer.size() != std::numeric_limits<std::uint16_t>::max());
    read_buffer.buffer.emplace_back(0);
    read_buffer.out_buffer.emplace_back(&data);

    state = State::reading;
}

/**
 * Write data to I2C device.
 *
 * Starts a new transaction if one is not already started. data is
 * actually written after transmit is called.
 *
 * @param[in] data Data to write to the I2C device.
 */
void Device::write(std::uint8_t data)
{
    switch (state) {
    case State::empty:
    case State::reading:
        message_buffers.emplace_back(WriteMessageBuffer{});
        break;
    case State::writing:
        break;
    default:
        assert(!"device transaction state not implemented");
    }

    auto& write_buffer = std::get<WriteMessageBuffer>(message_buffers.back());
    assert(write_buffer.buffer.size() != std::numeric_limits<std::uint16_t>::max());
    write_buffer.buffer.emplace_back(data);

    state = State::writing;
}

/**
 * Send complete transaction to I2C device.
 *
 * After transmission, all data read from the device is used to
 * populate previous calls to Device::read.
 *
 * @post Device transaction state is empty.
 * @post message_buffers is empty.
 */
void Device::transmit()
{
    transmitter->transmit(*this);

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
