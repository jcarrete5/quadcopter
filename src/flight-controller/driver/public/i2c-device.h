#ifndef FLIGHT_CONTROLLER_I2C_DEV_H
#define FLIGHT_CONTROLLER_I2C_DEV_H

#include <cstdint>
#include <memory>
#include <variant>
#include <vector>

namespace driver::i2c {
    /**
     * Implementation of an I2C device.
     *
     * @invariant The number of elements in message_buffers must be less than
     *            2^32 and non-negative.
     * @invariant The number of elements in a MessageBuffer must be less than
     *            2^16 and non-negative.
     */
    class Device {
    public:
        explicit Device(std::uint16_t address);
        Device(const Device&) = delete;
        Device(Device&&) = default;
        ~Device();

        Device& operator=(const Device&) = delete;
        Device& operator=(Device&&) = default;

        void read(std::uint8_t& data);
        void write(std::uint8_t data);
        void transmit();

    private:
        /// Forward declaration for platform-specific I2C message transmission.
        class MessageTransmitter;

        struct ReadMessageBuffer {
            /// Data buffer populated when reading data from an I2C device.
            std::vector<std::uint8_t> buffer;

            /// Stores locations to copy read data to after being populated in buffer.
            std::vector<std::uint8_t*> out_buffer;
        };

        struct WriteMessageBuffer {
            /// Data buffer used to construct a I2C message write segment.
            std::vector<std::uint8_t> buffer;
        };

        using MessageBuffer = std::variant<ReadMessageBuffer, WriteMessageBuffer>;

        /// Transaction state.
        enum class State {
            /// Transaction is empty.
            empty,
            /// Populating a read buffer for the current transaction.
            reading,
            /// Populating a write buffer for the current transaction.
            writing,
        };

        /// Platform-specific message transmitter implementation.
        std::unique_ptr<MessageTransmitter> transmitter;

        /// I2C address.
        std::uint16_t address;

        /// Current transaction state.
        State state;

        /// Buffers used to construct data to transmit.
        std::vector<MessageBuffer> message_buffers;
    };

    Device create_device(std::uint16_t address);
}// namespace driver::i2c

#endif// FLIGHT_CONTROLLER_I2C_DEV_H
