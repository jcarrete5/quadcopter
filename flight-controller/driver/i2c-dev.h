#ifndef FLIGHT_CONTROLLER_I2C_DEV_H
#define FLIGHT_CONTROLLER_I2C_DEV_H

#include <cstdint>
#include <string>
#include <vector>

/**
 * @brief I2C device driver abstraction.
 *
 * Manages the I2C bus file descriptor resource.
 */
class I2CDev {
public:
    explicit I2CDev(const std::string& i2c_bus, uint16_t address);
    ~I2CDev();
    I2CDev(const I2CDev& other);
    I2CDev(I2CDev&& other) = default;
    I2CDev& operator=(I2CDev other);
    I2CDev& operator=(I2CDev&& other) = default;

    friend void swap(I2CDev& first, I2CDev& second)
    {
        using std::swap;
        swap(first.bus_fd, second.bus_fd);
        swap(first.address, second.address);
    }

    [[nodiscard]] std::uint8_t read(std::uint8_t reg) const;
    [[nodiscard]] std::vector<std::uint8_t> read(std::uint8_t start_reg,
        std::uint16_t length) const;
    void write(std::uint8_t reg, std::uint8_t byte) const;
    void write(std::uint8_t reg, std::vector<uint8_t> buffer) const;

private:
    int bus_fd;             ///< I2C bus file descriptor.
    std::uint16_t address;  ///< I2C device address.
};

#endif  // FLIGHT_CONTROLLER_I2C_DEV_H
