#include "gtest/gtest.h"

#include "i2c-device.h"

using namespace driver;

class I2CDeviceTest : public ::testing::Test {
protected:
    I2CDeviceTest() : device{i2c::create_device(0x10)} {}

public:
    i2c::Device device;
};

TEST_F(I2CDeviceTest, WriteRegisterTransaction)
{
    constexpr std::uint8_t w0 = 1;
    constexpr std::uint8_t r_data[] = {1, 2, 3};

    device.write(w0);
    device.write(r_data[0]);
    device.write(r_data[1]);
    device.write(r_data[2]);
    device.transmit();

    std::uint8_t r[3];
    device.write(w0);
    device.read(r[0]);
    device.read(r[1]);
    device.read(r[2]);
    device.transmit();

    ASSERT_EQ(r_data[0], r[0]);
    ASSERT_EQ(r_data[1], r[1]);
    ASSERT_EQ(r_data[2], r[2]);
}
