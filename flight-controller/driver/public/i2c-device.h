#ifndef FLIGHT_CONTROLLER_I2C_DEV_H
#define FLIGHT_CONTROLLER_I2C_DEV_H

#include <cstdint>
#include <memory>

namespace driver::i2c {
    class Device {
    public:
        virtual ~Device() noexcept = default;

        virtual void read(uint8_t& data) = 0;
        virtual void write(uint8_t data) = 0;
        virtual void transmit() = 0;
    };

    std::unique_ptr<Device> create_device(std::uint16_t address);
}

#endif  // FLIGHT_CONTROLLER_I2C_DEV_H
