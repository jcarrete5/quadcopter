#ifndef FLIGHT_CONTROLLER_I2C_DEV_H
#define FLIGHT_CONTROLLER_I2C_DEV_H

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <tuple>
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
    I2CDev(const I2CDev& other) = delete;
    I2CDev& operator=(const I2CDev& other) = delete;
    I2CDev(I2CDev&& other) = default;
    I2CDev& operator=(I2CDev&& other) = default;

    [[nodiscard]] std::uint8_t read(std::uint8_t reg) const;
    [[nodiscard]] std::vector<std::uint8_t> read(std::uint8_t start_reg,
        std::uint16_t length) const;
    void write(std::uint8_t reg, std::uint8_t byte) const;
    void write(std::uint8_t reg, std::vector<uint8_t> buffer) const;

private:
    using I2CDevID = std::tuple<std::string, std::uint16_t>;  ///< Tuple of device path and I2C address.

    std::unique_lock<std::mutex> device_lock;
    int bus_fd;  ///< I2C bus file descriptor.
    std::uint16_t address;  ///< I2C device address.

    static std::unique_lock<std::mutex> lock_device(const I2CDevID& dev_id);
};

#endif  // FLIGHT_CONTROLLER_I2C_DEV_H
