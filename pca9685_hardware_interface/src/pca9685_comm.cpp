#include "pca9685_hardware_interface/pca9685_comm.h"
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "pca9685_hardware_interface/Constants.h"
#include "pca9685_hardware_interface/I2CPeripheral.h"

namespace PiPCA9685 {

PCA9685::PCA9685(const std::string &device, int address) {
  i2c_dev = std::make_unique<I2CPeripheral>(device, address);
}

void PCA9685::activate() {
  set_all_pwm(0, 0);
  i2c_dev->WriteRegisterByte(MODE2, OUTDRV);
  i2c_dev->WriteRegisterByte(MODE1, ALLCALL);
  usleep(5'000);

  auto mode1_val = i2c_dev->ReadRegisterByte(MODE1);
  mode1_val &= ~SLEEP;
  i2c_dev->WriteRegisterByte(MODE1, mode1_val);
  usleep(5'000);
}

void PCA9685::shutdown() {
  auto mode1_val = i2c_dev->ReadRegisterByte(MODE1);
  mode1_val |= SLEEP;
  i2c_dev->WriteRegisterByte(MODE1, mode1_val);
}

PCA9685::~PCA9685() {
  shutdown();
}

void PCA9685::set_pwm_freq(const double freq_hz) {
  frequency = freq_hz;

  auto prescaleval = 2.5e7; //    # 25MHz
  prescaleval /= 4096.0; //       # 12-bit
  prescaleval /= freq_hz;
  prescaleval -= 1.0;

  auto prescale = static_cast<int>(std::round(prescaleval));

  const auto oldmode = i2c_dev->ReadRegisterByte(MODE1);

  auto newmode = (oldmode & 0x7F) | SLEEP;

  i2c_dev->WriteRegisterByte(MODE1, newmode);
  i2c_dev->WriteRegisterByte(PRESCALE, prescale);
  i2c_dev->WriteRegisterByte(MODE1, oldmode);
  usleep(5'000);
  i2c_dev->WriteRegisterByte(MODE1, oldmode | RESTART);
}

void PCA9685::set_pwm(const int channel, const uint16_t on, const uint16_t off) {
  if (channel < 0 || channel >= 16) {
    throw std::out_of_range("PCA9685 channel index must be in [0, 15]");
  }
  const auto channel_offset = 4 * channel;
  i2c_dev->WriteRegisterByte(LED0_ON_L+channel_offset, on & 0xFF);
  i2c_dev->WriteRegisterByte(LED0_ON_H+channel_offset, on >> 8);
  i2c_dev->WriteRegisterByte(LED0_OFF_L+channel_offset, off & 0xFF);
  i2c_dev->WriteRegisterByte(LED0_OFF_H+channel_offset, off >> 8);
}

void PCA9685::set_all_pwm(const uint16_t on, const uint16_t off) {
  i2c_dev->WriteRegisterByte(ALL_LED_ON_L, on & 0xFF);
  i2c_dev->WriteRegisterByte(ALL_LED_ON_H, on >> 8);
  i2c_dev->WriteRegisterByte(ALL_LED_OFF_L, off & 0xFF);
  i2c_dev->WriteRegisterByte(ALL_LED_OFF_H, off >> 8);
}

void PCA9685::set_pwm_pulse_width_ms(const int channel, const double ms) {
  if (channel < 0 || channel >= 16) {
    throw std::out_of_range("PCA9685 channel index must be in [0, 15]");
  }
  const auto period_ms = 1000.0 / frequency;
  const auto constrained_ms = std::clamp(ms, 0.0, period_ms);
  const auto bits_per_ms = 4095.0 / period_ms;
  const auto raw_bits = constrained_ms * bits_per_ms;
  const auto bounded_bits = std::clamp(raw_bits, 0.0, 4095.0);
  set_pwm(channel, 0, static_cast<uint16_t>(bounded_bits));
}

}  // namespace PiPCA9685
