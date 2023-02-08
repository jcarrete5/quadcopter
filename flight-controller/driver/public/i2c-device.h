#ifndef FLIGHT_CONTROLLER_I2C_DEV_H
#define FLIGHT_CONTROLLER_I2C_DEV_H

#include <cstdint>
#include <memory>

namespace driver::i2c {
    class Device {
    public:
        virtual ~Device() noexcept = default;

        /**
         * Read data into argument from I2C device.
         *
         * Starts a new transaction if one is not already started. data will be
         * populated after transmit is called.
         *
         * @param[out] data Reference to populate after transmitting the complete
         *                  transaction.
         */
        virtual void read(uint8_t& data) = 0;

        /**
         * Write data to I2C device.
         *
         * Starts a new transaction if one is not already started. data is
         * actually written after transmit is called.
         *
         * @param[in] data Data to write to the I2C device.
         */
        virtual void write(uint8_t data) = 0;

        /**
         * Send complete transaction to I2C device.
         *
         * After transmission, all data read from the device is used to
         * populate previous calls to Device::read.
         */
        virtual void transmit() = 0;
    };

    std::unique_ptr<Device> create_device(std::uint16_t address);
}// namespace driver::i2c

#endif// FLIGHT_CONTROLLER_I2C_DEV_H
