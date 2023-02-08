#ifndef FLIGHT_CONTROLLER_I2C_DEVICE_H
#define FLIGHT_CONTROLLER_I2C_DEVICE_H

#include <vector>

#include <linux/i2c.h>
#include <variant>

#include "file-descriptor.h"
#include "public/i2c-device.h"

namespace driver::i2c {
    /**
     * Implementation of an I2C device using Linux-specific APIs.
     *
     * This implementation requires I2C_FUNC_I2C functionality to be available
     * for the configured I2C bus
     * (https://www.kernel.org/doc/html/v5.15/i2c/functionality.html).
     *
     * @invariant The number of elements in message_buffers must be less than
     *            2^32 and non-negative.
     * @invariant The number of elements in a MessageBuffer must be less than
     *            2^16 and non-negative.
     */
    class LinuxDevice : public Device {
    public:
        explicit LinuxDevice(std::uint16_t address);
        LinuxDevice(const LinuxDevice& other) = delete;
        LinuxDevice(LinuxDevice&& other) = default;
        ~LinuxDevice() noexcept override = default;
        LinuxDevice& operator=(const LinuxDevice& other) = delete;
        LinuxDevice& operator=(LinuxDevice&& other) = default;

        void read(std::uint8_t& data) override;
        void write(std::uint8_t data) override;
        void transmit() override;

    private:
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

        /// I2C address.
        std::uint16_t address;

        /// I2C bus file descriptor.
        util::FileDescriptor bus_fd;

        /// Current transaction state.
        State state;

        /// Buffers used to construct i2c_rdwr_ioctl_data during transmit.
        std::vector<MessageBuffer> message_buffers;

        [[nodiscard]] bool has_required_functionality() const;
    };
}// namespace driver::i2c

#endif// FLIGHT_CONTROLLER_I2C_DEVICE_H
