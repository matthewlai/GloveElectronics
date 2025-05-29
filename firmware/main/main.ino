#include <algorithm>
#include <utility>

#include <Wire.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

constexpr int LED_PIN = 21;
constexpr int SYNC_SERVER_CLIENT_SELECT_PIN = 5;  // high = server, low = client
constexpr int PWM_PINS[4] = { 0, 1, 4, 3 };

constexpr int TARGET_DRIVE_FREQUENCY = 250;  // 250 Hz according to Pfeifer et al. 2021

// The chip expects a PWM signal where duty cycle 50 means stopped, and 255 is full speed in one phase, 0 full speed in the opposite phase.
// Though for signal reasons we can't actually use 0% and 100%.
constexpr uint16_t ACTIVE_DUTY_CYCLE = 140;

constexpr float LOW_BATT_THRESHOLD = 3.2f;  // This is quite high, but lower than this and we start getting brown out resets at high power.

const uint32_t TOTAL_RUN_TIME_S = 2 * 60 * 60;  // 2 hours.

// Bluetooth Low Energy params.
// These are our randomly generated unique Service and Characteristic UUIDs, so the gloves can recognise each other.
constexpr char* SERVICE_UUID = "21014a2d-ebee-4fcb-b8d4-dcf34c19610a";

// The server sends its PRNG state every cycle so clients can sync up with it.
// 0 is a special value that means we are done.
constexpr char* CHARACTERISTIC_UUID = "44f21c6b-b08b-4695-970f-f21a15b538db";

constexpr int BLE_SCAN_TIME_S = 5;

constexpr int DRV_EN_PIN = 10;
constexpr int SDA_PIN = 8;
constexpr int SCL_PIN = 2;
constexpr uint16_t I2C_MUX_ADDR = 0x70;
constexpr uint16_t DRV2605_I2C_ADDR = 0x5A;
constexpr uint8_t NUM_CHANNELS = 4;

constexpr uint8_t DRV2605_REG_MODE = 0x01;
constexpr uint8_t DRV2605_REG_FEEDBACK = 0x1A;
constexpr uint8_t DRV2605_REG_CONTROL3 = 0x1D;
constexpr uint8_t DRV2605_REG_VBATT = 0x21;

void SetI2CChannel(uint8_t channel) {
  Wire.beginTransmission(I2C_MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void WriteDriverRegister(uint8_t reg_addr, uint8_t val) {
  uint8_t buf[2] = {reg_addr, val};
  Wire.beginTransmission(DRV2605_I2C_ADDR);
  Wire.write(buf, 2);
  Wire.endTransmission();
}

uint8_t ReadDriverRegister(uint8_t reg_addr) {
  uint8_t buf[1] = {reg_addr};
  Wire.beginTransmission(DRV2605_I2C_ADDR);
  Wire.write(buf, 1);
  Wire.endTransmission(/*stop=*/false);
  Wire.requestFrom(DRV2605_I2C_ADDR, 1);
  buf[0] = Wire.read();
  Wire.endTransmission();
  return buf[0];
}

void InitDriver(uint8_t channel) {
  digitalWrite(DRV_EN_PIN, HIGH);

  SetI2CChannel(channel);
  
  // Wait 250 microseconds for the driver to be ready (datasheet page 53).
  delayMicroseconds(250);

  // This init sequence is adapted from the Adafruit DRV2605 library.
  WriteDriverRegister(DRV2605_REG_MODE, 0x03);  // out of standby and into PWM input mode.

  WriteDriverRegister(DRV2605_REG_FEEDBACK, ReadDriverRegister(DRV2605_REG_FEEDBACK) | 0x80);  // LRA mode.
  WriteDriverRegister(DRV2605_REG_CONTROL3, ReadDriverRegister(DRV2605_REG_CONTROL3) | 0x01);  // LRA open loop mode.
}

float ReadVBatt() {
  // Use the currently selected driver to read battery voltage.
  return static_cast<float>(ReadDriverRegister(DRV2605_REG_VBATT)) * 5.6f / 255.0f;
}

// For reproducible stimulation patterns (so we can give two hands the same pattern), we need
// a PRNG that has a small state that can be shared. It doesn't need to be cryptographically secure.
// We implement the linear congruential generator here, with C++11's minstd_rand params.
constexpr uint32_t LCG_M = (1UL << 31) - 1;
constexpr uint32_t LCG_A = 48271;
constexpr uint32_t LCG_C = 0;
uint32_t rng_state;

void SeedPRNG(uint32_t seed) {
  rng_state = seed;
}

uint32_t PRNG() {
  rng_state = (LCG_A * rng_state + LCG_C) % LCG_M;
  // We don't need a large range, and LCG has low period in low bits, so
  // we throw away some low bits.
  return rng_state >> 8;
}

void MakePermutatedSequence(uint8_t buf_out[NUM_CHANNELS], bool reversed) {
  for (int i = 0; i < NUM_CHANNELS; ++i) {
    buf_out[i] = i;
  }
  if (NUM_CHANNELS <= 1) {
    return;
  }
  for (int i = 0; i < (NUM_CHANNELS - 1); ++i) {
    std::swap(buf_out[i], buf_out[PRNG() % NUM_CHANNELS]);
  }
  if (reversed) {
    std::reverse(buf_out, buf_out + NUM_CHANNELS);
  }
}

bool is_ble_server = false;

// For BLE server.
BLEServer* server = nullptr;
BLEService* service = nullptr;
BLECharacteristic* characteristic = nullptr;
BLEAdvertising* advertising = nullptr;
bool last_device_connected = false;
volatile bool device_connected = false;

// For BLE client.
volatile bool connected_to_server = false;
volatile bool have_new_value = false;
volatile uint32_t new_value_time = 0;
BLEScan* scan = nullptr;
BLEClient* client = nullptr;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer*) {
      device_connected = true;
    };
    void onDisconnect(BLEServer*) {
      device_connected = false;
    }
};

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {}

  void onDisconnect(BLEClient* pclient) {
    connected_to_server = false;
  }
};

static void ClientNotifyCallback(
  BLERemoteCharacteristic* remote_characteristic,
  uint8_t* data,
  size_t len,
  bool is_notify
) {
  if (len != 4) {
    Serial.printf("Wrong data length: %u, expected 4", len);
    return;
  }
  memcpy(&rng_state, data, len);
  new_value_time = millis();
  have_new_value = true;
}

void setup() {
  Serial.begin();
  delay(500);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DRV_EN_PIN, OUTPUT);
  pinMode(SYNC_SERVER_CLIENT_SELECT_PIN, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  for (int i = 0; i < NUM_CHANNELS; ++i) {
    pinMode(PWM_PINS[i], OUTPUT);

    // PWM frequency is drive frequency * 128 (DRV2605 datasheet page 14).
    ledcAttach(PWM_PINS[i], TARGET_DRIVE_FREQUENCY * 128, 8);
    InitDriver(i);
  }

  is_ble_server = digitalRead(SYNC_SERVER_CLIENT_SELECT_PIN) == HIGH;
  if (is_ble_server) {
    Serial.println("Starting as BLE server");
    BLEDevice::init("Vibrating Glove (Server)");
    server = BLEDevice::createServer();
    server->setCallbacks(new MyServerCallbacks());
    service = server->createService(SERVICE_UUID);
    characteristic = service->createCharacteristic(CHARACTERISTIC_UUID,
                                                   BLECharacteristic::PROPERTY_READ |
                                                   BLECharacteristic::PROPERTY_NOTIFY);
    uint32_t initial_value = 42;
    characteristic->setValue(initial_value);
    service->start();

    advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMaxPreferred(12);
    advertising->start();
  } else {
    Serial.println("Starting as BLE client");
    BLEDevice::init("Vibrating Glove (Client)");
    scan = BLEDevice::getScan();
    //scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);
    client = BLEDevice::createClient();
  }

  SeedPRNG(42);
}

void Shutdown(bool with_flashing_leds = true) {
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
    delay(100);  // Wait for any potentially in-flight I2C transmissions to finish.
  }
  
  // Go into permanent deep sleep, which is the lowest power state we can access from
  // firmware.
  esp_deep_sleep_start();
}

int low_battery_iterations = 0;

void loop() {
  float vbatt = ReadVBatt();
  if (vbatt < LOW_BATT_THRESHOLD) {
    ++low_battery_iterations;
    if (low_battery_iterations > 3) {
      Serial.printf("Low battery: %fV. Shutting down", vbatt);
      Shutdown();
    }
  } else {
    low_battery_iterations = 0;
  }

  if (millis() > (TOTAL_RUN_TIME_S * 1000)) {
    Serial.println("We are done! Shutting down now");
    Shutdown(false);
  }

  if (is_ble_server) {
    if (device_connected && !last_device_connected) {
      // We have a new connection, and can stop advertising.
      advertising->stop();
    }
    if (!device_connected && last_device_connected) {
      // Client has disconnected. Start advertising again.
      advertising->start();
    }
    if (device_connected) {
      characteristic->setValue(rng_state);
      characteristic->notify();
    }
    last_device_connected = device_connected;
  } else {
    if (!connected_to_server) {
      BLEScanResults* found_devices = scan->start(BLE_SCAN_TIME_S, false);
      Serial.print("Devices found: ");
      Serial.println(found_devices->getCount());
      for (int dev_id = 0; dev_id < found_devices->getCount(); ++dev_id) {
        auto remote_device = found_devices->getDevice(dev_id);
        BLEUUID service_uuid = remote_device.getServiceUUID();
        if (service_uuid.equals(BLEUUID(SERVICE_UUID))) {
          Serial.println("Found our device! Connecting");
          client->setClientCallbacks(new MyClientCallback());
          client->connect(&remote_device);
          BLERemoteService* remote_service = client->getService(SERVICE_UUID);
          if (remote_service == nullptr) {
            Serial.println("We don't have the right service?");
            client->disconnect();
            break;
          }
          BLERemoteCharacteristic* remote_characteristic = remote_service->getCharacteristic(CHARACTERISTIC_UUID);
          if (remote_characteristic == nullptr) {
            Serial.println("We don't have the right characteristic?");
            client->disconnect();
            break;
          }
          if (!remote_characteristic->canRead() || !remote_characteristic->canNotify()) {
            Serial.println("Wrong characteristic capabilities set on server");
            client->disconnect();
            break;
          }
          remote_characteristic->registerForNotify(ClientNotifyCallback);
          connected_to_server = true;
          Serial.printf("We are connected! Current seed: %u\n", remote_characteristic->readUInt32());
        }
      }
      scan->clearResults();
      delay(2000);
    }
    if (!connected_to_server) {
      return;
    } else {
      // Wait for a sync point. We are doing a fast busy loop here to minimize latency.
      while (true) {
        if (have_new_value) {
          uint32_t new_value_freshness = millis() - new_value_time;
          // If we have a fresh value, we are done, otherwise discard and wait for the next sync point.
          if (new_value_freshness < 20) {
            break;
          } else {
            have_new_value = false;
          }
        }
        delayMicroseconds(100);
      }
      have_new_value = false;
      if (rng_state == 0) {
        Serial.println("Remote glove says we are done. Shutting down.");
        Shutdown();
      }
    }
  }

  uint8_t seq[NUM_CHANNELS];
  MakePermutatedSequence(seq, /*reversed=*/!is_ble_server);
  for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
    ledcWrite(PWM_PINS[seq[ch]], ACTIVE_DUTY_CYCLE);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    ledcWrite(PWM_PINS[seq[ch]], 128);
    digitalWrite(LED_PIN, LOW);
    delay(66);
  }
  delay(666);
}
