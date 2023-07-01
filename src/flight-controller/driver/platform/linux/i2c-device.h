#ifndef FLIGHT_CONTROLLER_I2C_DEVICE_H
#define FLIGHT_CONTROLLER_I2C_DEVICE_H

#include <vector>

#include <linux/i2c.h>
#include <variant>

#include "file-descriptor.h"
#include "public/i2c-device.h"

namespace driver::i2c {
    /**
     * This implementation requires I2C_FUNC_I2C functionality to be available
     * for the configured I2C bus:
     *     https://www.kernel.org/doc/html/v5.15/i2c/functionality.html
     */
    class Device::MessageTransmitter {
    public:
        MessageTransmitter();

        void transmit(Device& device);

    private:
        /// I2C bus file descriptor.
        util::FileDescriptor bus_fd;

        [[nodiscard]] bool has_required_functionality() const;
    };
}// namespace driver::i2c

#endif// FLIGHT_CONTROLLER_I2C_DEVICE_H
