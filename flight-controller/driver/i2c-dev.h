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
 * Manages the I2C bus file descriptor resource and provides an interface for
 * communicating with I2C devices.
 *
 * @invariant I2C bus file descriptor is open and ready for use.
 * @note Unfortunately, it is impossible to guarantee that an I2C device always
 * remains connected to the bus. On construction, the presence of the I2C device
 * is verified but it is possible that the device could be removed at a later
 * time. You can use the I2CDev::device_exists member function to check for
 * presence of the device.
 */
class I2CDev {
public:
    /// Tuple of device path and I2C address.
    using I2CDevID = std::tuple<std::string, std::uint16_t>;

    explicit I2CDev(const std::string& i2c_bus, uint16_t address);
    ~I2CDev();
    I2CDev(const I2CDev& other) = delete;
    I2CDev& operator=(const I2CDev& other) = delete;
    I2CDev(I2CDev&& other) = default;
    I2CDev& operator=(I2CDev&& other) = default;

    [[nodiscard]] bool device_exists() const;

    [[nodiscard]] std::uint8_t read(std::uint8_t reg) const;
    [[nodiscard]] std::vector<std::uint8_t> read(std::uint8_t start_reg,
        std::uint16_t length) const;
    void write(std::uint8_t reg, std::uint8_t byte) const;
    void write(std::uint8_t reg, std::vector<std::uint8_t> buffer) const;

private:
    /// Device lock to ensure exclusive access to the device.
    std::unique_lock<std::mutex> device_lock;
    /// I2C bus file descriptor.
    int bus_fd;
    /// I2C device address.
    std::uint16_t address;

    static std::unique_lock<std::mutex> lock_device(const I2CDevID& dev_id);
};

#endif  // FLIGHT_CONTROLLER_I2C_DEV_H
