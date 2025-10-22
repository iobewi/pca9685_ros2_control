#include <gtest/gtest.h>

#include <cerrno>
#include <cstring>
#include <system_error>

extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#define private public
#include "pca9685_hardware_interface/I2CPeripheral.h"
#undef private

extern "C" {

int open(const char *, int, ...) {
  return 123;
}

int ioctl(int, unsigned long, ...) {
  return 0;
}

int close(int) {
  return 0;
}

int i2c_smbus_access(int, char, __u8, int, union i2c_smbus_data *) {
  errno = ENXIO;
  return -1;
}

}  // extern "C"

TEST(I2CPeripheralErrorHandlingTest, PropagatesErrnoOnReadFailure) {
  PiPCA9685::I2CPeripheral peripheral{"/dev/i2c-mock", 0x01};

  try {
    (void)peripheral.ReadRegisterByte(0x00);
    FAIL() << "Expected std::system_error";
  } catch(const std::system_error &error) {
    EXPECT_EQ(error.code().value(), ENXIO);
    EXPECT_EQ(&error.code().category(), &std::system_category());

    const std::string message{error.what()};
    EXPECT_NE(message.find("Could not read value at register 0"), std::string::npos);
    EXPECT_NE(message.find(std::strerror(ENXIO)), std::string::npos);
  }
}
