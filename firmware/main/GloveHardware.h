#pragma once
#include "GloveApp.h"
#include "config.h"
#include <Wire.h>

class I2CMux {
public:
  static I2CMux &getInstance() {
    static I2CMux instance;
    return instance;
  }
  I2CMux(const I2CMux &) = delete;
  I2CMux &operator=(const I2CMux &) = delete;

  void selectChannel(uint8_t channel) {
    if (channel != current_channel_) {
      Wire.beginTransmission(I2C_MUX_ADDR);
      Wire.write(1 << channel);
      Wire.endTransmission();
      current_channel_ = channel;
    }
  }

private:
  I2CMux() : current_channel_(-1) {
    digitalWrite(DRV_EN_PIN, HIGH);
    Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_FREQUENCY);
    // Wait for the driver to be ready (datasheet page 53).
    delayMicroseconds(250);
  }
  int current_channel_;
};

class DRV2605 {
public:
  DRV2605(uint8_t channel) : channel_(channel) {
    // This init sequence is adapted from the Adafruit DRV2605 library.
    // Puts the driver out of standby and into PWM input mode.
    setMode(MODE_PWM);
    setFeedback(getFeedback() | LRA_MODE);
    setControl3(getControl3() | LRA_OPEN_LOOP);
  }

  float getVBatt() const {
    I2CMux::getInstance().selectChannel(channel_);
    return static_cast<float>(_readRegister(REG_VBATT)) * 5.6f / 255.0f;
  }

  void setMode(uint8_t mode) {
    I2CMux::getInstance().selectChannel(channel_);
    _writeRegister(REG_MODE, mode);
  }

  uint8_t getMode() const {
    I2CMux::getInstance().selectChannel(channel_);
    return _readRegister(REG_MODE);
  }

  void setFeedback(uint8_t value) {
    I2CMux::getInstance().selectChannel(channel_);
    _writeRegister(REG_FEEDBACK, value);
  }

  uint8_t getFeedback() const {
    I2CMux::getInstance().selectChannel(channel_);
    return _readRegister(REG_FEEDBACK);
  }

  void setControl3(uint8_t value) {
    I2CMux::getInstance().selectChannel(channel_);
    _writeRegister(REG_CONTROL3, value);
  }

  uint8_t getControl3() const {
    I2CMux::getInstance().selectChannel(channel_);
    return _readRegister(REG_CONTROL3);
  }

private:
  void _writeRegister(uint8_t reg_addr, uint8_t val) {
    uint8_t buf[2] = {reg_addr, val};
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(buf, 2);
    Wire.endTransmission();
  }

  uint8_t _readRegister(uint8_t reg_addr) const {
    uint8_t buf[1] = {reg_addr};
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(buf, 1);
    Wire.endTransmission(/*stop=*/false);
    Wire.requestFrom(I2C_ADDR, (uint8_t)1);
    buf[0] = Wire.read();
    Wire.endTransmission();
    return buf[0];
  }

  const uint8_t channel_;

  static constexpr uint16_t I2C_ADDR = 0x5A;
  static constexpr uint8_t REG_MODE = 0x01;
  static constexpr uint8_t MODE_PWM = 0x03;
  static constexpr uint8_t REG_FEEDBACK = 0x1A;
  static constexpr uint8_t REG_CONTROL3 = 0x1D;
  static constexpr uint8_t REG_VBATT = 0x21;
  static constexpr uint8_t LRA_MODE = 0x80;
  static constexpr uint8_t LRA_OPEN_LOOP = 0x01;
};
class Motor {
public:
  Motor(int pwm_pin, int channel) : pwm_pin_(pwm_pin), driver_(channel) {
    pinMode(pwm_pin_, OUTPUT);
    // PWM frequency is drive frequency * 128 (DRV2605 datasheet page 14).
    ledcAttach(pwm_pin_, TARGET_DRIVE_FREQUENCY * 128, 8);
  }

  void on() { ledcWrite(pwm_pin_, ACTIVE_DUTY_CYCLE); }
  void off() { ledcWrite(pwm_pin_, INACTIVE_DUTY_CYCLE); }

  DRV2605 &getDriver() { return driver_; }
  const DRV2605 &getDriver() const { return driver_; }

private:
  const int pwm_pin_;
  DRV2605 driver_;
};

class Glove {
public:
  static Glove &getInstance() {
    static Glove instance;
    return instance;
  }
  Glove(const Glove &) = delete;
  Glove &operator=(const Glove &) = delete;

  void init() {}

  void shutdown(bool with_flashing_leds = true) {
    digitalWrite(DRV_EN_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    if (with_flashing_leds) {
      for (int i = 0; i < 3; ++i) {
        digitalWrite(LED_PIN, HIGH);
        delay(300);
        digitalWrite(LED_PIN, LOW);
        delay(300);
      }
    } else {
      delay(100);
    }
    esp_deep_sleep_start();
  }

  bool isBatteryOK() {
    static int low_battery_iterations = 0;
    vbatt_ = motors_[0].getDriver().getVBatt();
    if (vbatt_ >= LOW_BATT_THRESHOLD) {
      low_battery_iterations = 0;
      return true;
    }
    return ++low_battery_iterations < 4;
  }

  float getBatteryVoltage() const { return vbatt_; }
  void setCurrentPattern(int pattern) { current_pattern_ = pattern; }
  int getCurrentPattern() const { return current_pattern_; }
  static constexpr unsigned int getNumPatterns() {
    return NUM_VIBRATION_PATTERNS;
  }

  void runMotors(bool is_client) {
    int pattern = current_pattern_;
    Serial.printf("%010u Running pattern #%02d\n", xTaskGetTickCount(),
                  pattern);

    if (pattern < 0) {
      vTaskDelay(pdMS_TO_TICKS(ACTIVE_CYCLE_DURATION));
      return;
    }

    uint32_t start_time = millis();
    uint8_t seq = vibration_patterns_[pattern];

    if (is_client) {
      seq = ~seq;
      digitalWrite(SYNC_PIN, HIGH);
    }
    Serial.printf("%010u Sequence 0x%02x\n", xTaskGetTickCount(), seq);

    for (int ch = 0; ch < NUM_MOTORS; ++ch, seq >>= 2) {
      int finger = seq & 0x03;
      motors_[finger].on();
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(PULSE_DURATION);
      motors_[finger].off();
      digitalWrite(LED_PIN, LOW);

      if (ch == 0) {
        if (is_client) {
          digitalWrite(SYNC_PIN, LOW);
        } else if (GloveApp::getInstance().sync_time != 0) {
          Serial.printf("%010u Skew %dms\n", xTaskGetTickCount(),
                        start_time - GloveApp::getInstance().sync_time);
        }
      }
      vTaskDelay(PAUSE_DURATION);
    }
    Serial.printf("%010u Done pattern #%02d\n", xTaskGetTickCount(), pattern);
  }

private:
  Glove() : motors_{Motor(0, 0), Motor(1, 1), Motor(4, 2), Motor(3, 3)}, vbatt_(0.0f), current_pattern_(-1) {}

  Motor motors_[4];
  static constexpr int NUM_MOTORS = 4;
  static constexpr uint8_t vibration_patterns_[] = {
      VIBRATION_PATTERNS(0, 1, 2, 3)};
  static constexpr unsigned int NUM_VIBRATION_PATTERNS =
      sizeof(vibration_patterns_) / sizeof(vibration_patterns_[0]);

  float vbatt_;
  volatile int current_pattern_;
};
