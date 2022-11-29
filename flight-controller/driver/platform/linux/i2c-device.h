#ifndef FLIGHT_CONTROLLER_I2C_DEVICE_H
#define FLIGHT_CONTROLLER_I2C_DEVICE_H

#include <vector>

#include <linux/i2c.h>

#include "file-descriptor.h"
#include "public/i2c-device.h"

namespace driver::i2c {
    class LinuxDevice : public Device {
    public:
        LinuxDevice() = delete;
        explicit LinuxDevice(std::uint16_t address);
        LinuxDevice(const LinuxDevice& other) = delete;
        LinuxDevice(LinuxDevice&& other) = default;
        LinuxDevice& operator=(const LinuxDevice& other) = delete;
        LinuxDevice& operator=(LinuxDevice&& other) = default;

        void read(std::uint8_t& data) override;
        void write(std::uint8_t data) override;
        void transmit() override;

    private:
        struct MessageBuffer {
            enum class Direction {
                read,
                write,
            };

            Direction direction;
            i2c_msg* message;
            std::vector<std::uint8_t> buffer;
            std::vector<std::uint8_t*> out_buffer;

            explicit MessageBuffer(Direction direction, i2c_msg &i2c_message);

            void append(std::uint8_t data);
            void append(std::uint8_t* data);
        };

        enum class State {
            empty,
            reading,
            writing,
        };

        /// I2C address.
        std::uint16_t address;

        /// I2C bus file descriptor.
        util::FileDescriptor bus_fd;

        State state;
        std::vector<MessageBuffer> message_buffers;
        std::vector<i2c_msg> messages;
    };
}

#endif  // FLIGHT_CONTROLLER_I2C_DEVICE_H
