// Select ESP32C3 Dev Module as the board, and enable "USB CDC On Boot" in the "Tools" menu.

#include <algorithm>
#include <utility>

#include <Wire.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

constexpr int LED_PIN = 21;
constexpr int SYNC_SERVER_CLIENT_SELECT_PIN = 5;  // high = server, low = client
constexpr int PWM_PINS[4] = { 0, 1, 4, 3 };

//**********************************************************************************
// I2C device driver

constexpr int DRV_EN_PIN = 10;
constexpr int SDA_PIN = 8;
constexpr int SCL_PIN = 2;
constexpr uint16_t I2C_MUX_ADDR = 0x70;
constexpr uint32_t I2C_CLOCK_FREQUENCY = 400000;
constexpr uint16_t DRV2605_I2C_ADDR = 0x5A;
constexpr uint8_t NUM_CHANNELS = 4;

constexpr uint8_t DRV2605_REG_MODE = 0x01;
constexpr uint8_t DRV2605_REG_FEEDBACK = 0x1A;
constexpr uint8_t DRV2605_REG_CONTROL3 = 0x1D;
constexpr uint8_t DRV2605_REG_VBATT = 0x21;

void I2CBegin() {
  Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_FREQUENCY);
}

void SetI2CChannel(uint8_t channel) {
  Wire.beginTransmission(I2C_MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void WriteDriverRegister(uint8_t reg_addr, uint8_t val) {
  uint8_t buf[2] = { reg_addr, val };
  Wire.beginTransmission(DRV2605_I2C_ADDR);
  Wire.write(buf, 2);
  Wire.endTransmission();
}

uint8_t ReadDriverRegister(uint8_t reg_addr) {
  uint8_t buf[1] = { reg_addr };
  Wire.beginTransmission(DRV2605_I2C_ADDR);
  Wire.write(buf, 1);
  Wire.endTransmission(/*stop=*/false);
  Wire.requestFrom(DRV2605_I2C_ADDR, 1);
  buf[0] = Wire.read();
  Wire.endTransmission();
  return buf[0];
}

//**********************************************************************************
// Power management

const uint32_t TOTAL_RUN_TIME_S = 2 * 60 * 60;  // 2 hours.

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

float ReadVBatt() {
  // Use the currently selected driver to read battery voltage.
  return static_cast<float>(ReadDriverRegister(DRV2605_REG_VBATT)) * 5.6f / 255.0f;
}

constexpr float LOW_BATT_THRESHOLD = 3.2f;  // This is quite high, but lower than this and we start getting brown out resets at high power.
float vbatt;

bool BatteryOK() {
  static int low_battery_iterations = 0;
  vbatt = ReadVBatt();
  if (vbatt >= LOW_BATT_THRESHOLD) {
    low_battery_iterations = 0;
    return true;
  }
  return ++low_battery_iterations < 4;
}

//**********************************************************************************
// Sequence control
#define VIBRATION_PATTERN(a, b, c, d) ((uint8_t)(((a)&0x03) | (((b)&0x03) << 2) | (((c)&0x03) << 4) | (((d)&0x03) << 6)))

// All possible sequences
constexpr uint8_t vibration_patterns[] = {
  VIBRATION_PATTERN(0, 1, 2, 3),
  VIBRATION_PATTERN(0, 1, 3, 2),
  VIBRATION_PATTERN(0, 2, 1, 3),
  VIBRATION_PATTERN(0, 2, 3, 1),
  VIBRATION_PATTERN(0, 3, 1, 2),
  VIBRATION_PATTERN(0, 3, 2, 1),
  VIBRATION_PATTERN(1, 0, 2, 3),
  VIBRATION_PATTERN(1, 0, 3, 2),
  VIBRATION_PATTERN(1, 2, 0, 3),
  VIBRATION_PATTERN(1, 2, 3, 0),
  VIBRATION_PATTERN(1, 3, 0, 2),
  VIBRATION_PATTERN(1, 3, 2, 0),
  VIBRATION_PATTERN(2, 0, 1, 3),
  VIBRATION_PATTERN(2, 0, 3, 1),
  VIBRATION_PATTERN(2, 1, 0, 3),
  VIBRATION_PATTERN(2, 1, 3, 0),
  VIBRATION_PATTERN(2, 3, 0, 1),
  VIBRATION_PATTERN(2, 3, 1, 0),
  VIBRATION_PATTERN(3, 0, 1, 2),
  VIBRATION_PATTERN(3, 0, 2, 1),
  VIBRATION_PATTERN(3, 1, 0, 2),
  VIBRATION_PATTERN(3, 1, 2, 0),
  VIBRATION_PATTERN(3, 2, 0, 1),
  VIBRATION_PATTERN(3, 2, 1, 0),
};

//**********************************************************************************
// Motor control

constexpr int TARGET_DRIVE_FREQUENCY = 250;  // 250 Hz according to Pfeifer et al. 2021

// The chip expects a PWM signal where duty cycle 50 means stopped, and 255 is full speed in one phase, 0 full speed in the opposite phase.
// Though for signal reasons we can't actually use 0% and 100%.
// This should ideally be configurable at run time, we should figure out a UI (maybe a phone app?)
constexpr uint16_t ACTIVE_DUTY_CYCLE = 250;

volatile int current_pattern = -1;

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

void RunMotors(int pattern, bool mirrored) {
  if (pattern < 0) return;

  uint8_t seq = vibration_patterns[pattern];
  if (mirrored) seq = ~seq;

  Serial.printf("%010u Running sequence %02x\n", millis(), seq);
  TickType_t last_wake_time = xTaskGetTickCount();

  for (int ch = 0; ch < NUM_CHANNELS; ++ch, seq >>= 2) {
    int finger = seq & 0x03;
    ledcWrite(PWM_PINS[finger], ACTIVE_DUTY_CYCLE);
    digitalWrite(LED_PIN, HIGH);
    vTaskDelayUntil(&last_wake_time, 100 / portTICK_PERIOD_MS);
    ledcWrite(PWM_PINS[finger], 128);
    digitalWrite(LED_PIN, LOW);
    vTaskDelayUntil(&last_wake_time, 66 / portTICK_PERIOD_MS);
  }
}

//**********************************************************************************
// Bluetooth

// Bluetooth Low Energy params.
// These are our randomly generated unique Service and Characteristic UUIDs, so the gloves can recognise each other.
constexpr char* SERVICE_UUID = "21014a2d-ebee-4fcb-b8d4-dcf34c19610a";

// The server sends its clock and pattern to execute to the client so they remain in sync.
constexpr char* CHARACTERISTIC_UUID = "44f21c6b-b08b-4695-970f-f21a15b538db";

constexpr int BLE_SCAN_TIME_S = 5;

// For BLE server.
BLEServer* server = nullptr;
BLEService* service = nullptr;
BLECharacteristic* characteristic = nullptr;
BLEAdvertising* advertising = nullptr;
volatile bool last_device_connected = false;
volatile bool device_connected = false;
volatile TickType_t next_motor_cycle_start;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) {
    device_connected = true;
    Serial.println("Client connected");
  };
  void onDisconnect(BLEServer*) {
    device_connected = false;
    Serial.println("Client disconnected");
  }
};

void StartBLEServer() {
  BLEDevice::init("Vibrating Glove (Server)");
  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  service = server->createService(SERVICE_UUID);
  characteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  uint32_t data[] = {xTaskGetTickCount(), next_motor_cycle_start, current_pattern};
  characteristic->setValue((uint8_t *)data, sizeof(data));
  service->start();

  advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);  // This is apparently required for iPhones? Do we care?
  advertising->setMaxPreferred(0x12);
  advertising->start();
}

void BLEServerLoop() {
  if (device_connected && !last_device_connected) {
    // We have a new connection, and can stop advertising.
    advertising->stop();
  }
  if (!device_connected && last_device_connected) {
    // Client has disconnected. Start advertising again.
    advertising->start();
  }
  if (device_connected) {
    uint32_t data[] = {xTaskGetTickCount(), next_motor_cycle_start, current_pattern};
    characteristic->setValue((uint8_t *)data, sizeof(data));
    characteristic->notify();
  }
  last_device_connected = device_connected;
}

// For BLE client.
volatile bool connected_to_server = false;
volatile int32_t tick_drift;
BLEScan* scan = nullptr;
BLEClient* client = nullptr;

class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("Connected to server");
  }

  void onDisconnect(BLEClient* pclient) {
    connected_to_server = false;
    Serial.println("Disconnected from server");
  }
};

static void ClientNotifyCallback(BLERemoteCharacteristic* rc, uint8_t* data, size_t len, bool is_notify) {
  uint32_t value[3];
  if (len != sizeof(value)) {
    Serial.printf("Wrong data length: %u, expected %d", len, sizeof(value));
    return;
  }

  memcpy(&value, data, len);
  TickType_t server_ticks = value[0];
  TickType_t next_cycle_start = value[1];
  current_pattern = value[2];

  TickType_t client_ticks = xTaskGetTickCount();
  tick_drift = client_ticks - server_ticks;
  next_motor_cycle_start = next_cycle_start + tick_drift; 
  Serial.printf("%010u: New pattern starting at %010u(%u) %d\n", client_ticks, next_motor_cycle_start, tick_drift, current_pattern);
}

void StartBLEClient() {
  BLEDevice::init("Vibrating Glove (Client)");
  scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(0x10);  // 10ms.
  scan->setWindow(0x10);    // 10ms.
  client = BLEDevice::createClient();
}

bool BLEConnectToServer() {
  static ClientCallbacks *client_callbacks = nullptr; 

  if (connected_to_server) return true;

  if (!client_callbacks) client_callbacks = new ClientCallbacks();

  BLEScanResults* found_devices = scan->start(BLE_SCAN_TIME_S, false);
  Serial.print("Devices found: ");
  Serial.println(found_devices->getCount());
  for (int dev_id = 0; dev_id < found_devices->getCount(); ++dev_id) {
    auto remote_device = found_devices->getDevice(dev_id);
    BLEUUID service_uuid = remote_device.getServiceUUID();
    if (service_uuid.equals(BLEUUID(SERVICE_UUID))) {
      Serial.println("Found a device! Validating...");
      client->setClientCallbacks(client_callbacks);
      client->connect(&remote_device);
      BLERemoteService* remote_service = client->getService(SERVICE_UUID);  //do we ever need to free this?
      if (remote_service == nullptr) {
        Serial.println("We don't have the right service?");
        client->disconnect();
        continue;
      }
      BLERemoteCharacteristic* remote_characteristic = remote_service->getCharacteristic(CHARACTERISTIC_UUID);
      if (remote_characteristic == nullptr) {
        Serial.println("We don't have the right characteristic?");
        client->disconnect();
        continue;
      }
      if (!remote_characteristic->canRead() || !remote_characteristic->canNotify()) {
        Serial.println("Wrong characteristic capabilities set on server");
        client->disconnect();
        continue;
      }
      remote_characteristic->registerForNotify(ClientNotifyCallback);
      connected_to_server = true;
      break;
    }
  }
  scan->clearResults();
  return connected_to_server;
}

void BLEClientLoop() {
  if (!(connected_to_server || BLEConnectToServer())) return;
}


//**********************************************************************************
// Setup & Loop

const TickType_t ble_update_frequency = 100 / portTICK_PERIOD_MS;

void ble_server_task(void *parameter) {
  TickType_t last_wake_time = xTaskGetTickCount();

  StartBLEServer();
  while (true) {
    vTaskDelayUntil(&last_wake_time, ble_update_frequency);
    BLEServerLoop();
  }
}

void ble_client_task(void *parameter) {
  StartBLEClient();
  while (true) {
    BLEClientLoop();
    vTaskDelay(ble_update_frequency/2);
  }
}

const TickType_t pattern_frequency = 1000 / portTICK_PERIOD_MS;

void motor_server_task(void *parameter) {
  TickType_t last_wake_time = xTaskGetTickCount();

  while (true) {
    int pattern = current_pattern = random(sizeof(vibration_patterns) / sizeof(vibration_patterns[0]));
    next_motor_cycle_start = last_wake_time + pattern_frequency;
    vTaskDelayUntil(&last_wake_time, pattern_frequency);
    RunMotors(pattern, false);
  }
}

void motor_client_task(void *parameter) {
  while (next_motor_cycle_start <= xTaskGetTickCount() ) vTaskDelay(10 / portTICK_PERIOD_MS);

  while (true) {
    int gap = next_motor_cycle_start - xTaskGetTickCount();
    if (gap > 0) vTaskDelay(gap);
    int pattern = current_pattern;
    RunMotors(pattern, true);
  }
}

constexpr uint32_t TASK_STACK_SIZE = 10000;
TaskHandle_t ble_task_handle;
TaskHandle_t motor_task_handle;

void setup() {
  Serial.begin();
  delay(500);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DRV_EN_PIN, OUTPUT);
  pinMode(SYNC_SERVER_CLIENT_SELECT_PIN, INPUT_PULLUP);
  I2CBegin();

  for (int i = 0; i < NUM_CHANNELS; ++i) {
    pinMode(PWM_PINS[i], OUTPUT);

    // PWM frequency is drive frequency * 128 (DRV2605 datasheet page 14).
    ledcAttach(PWM_PINS[i], TARGET_DRIVE_FREQUENCY * 128, 8);
    InitDriver(i);
  }

  bool is_ble_server = digitalRead(SYNC_SERVER_CLIENT_SELECT_PIN) == HIGH;

  Serial.printf("Starting %s tasks: ", is_ble_server ? "server" : "client");
  if (is_ble_server) {
    xTaskCreate(motor_server_task, "Motor task", TASK_STACK_SIZE, NULL, 10, &motor_task_handle);
    xTaskCreate(ble_server_task, "BLE task", TASK_STACK_SIZE, NULL, 10, &ble_task_handle);
  } else {
    xTaskCreate(motor_client_task, "Motor task", TASK_STACK_SIZE, NULL, 10, &motor_task_handle);
    xTaskCreate(ble_client_task, "BLE task", TASK_STACK_SIZE, NULL, 10, &ble_task_handle);
  }
  Serial.println(motor_task_handle && ble_task_handle ? "SUCCESS" : "FAILURE");
}

// Low-priority housekeeping
void loop() {
  if (!BatteryOK()) {
    Serial.printf("Low battery (%fv). Shutting down\n", vbatt);
    Shutdown();
  }

  if (millis() > (TOTAL_RUN_TIME_S * 1000)) {
    Serial.println("We are done! Shutting down now");
    Shutdown(false);
  }
}
