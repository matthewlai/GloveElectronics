#pragma once
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "config.h"
#include "GloveHardware.h"
#include "GloveEspNow.h"

typedef struct __attribute__((packed)) PatternSyncMessage {
  TickType_t next_cycle_start;
  int pattern;
} PatternSyncMessage;

class GloveBLEServer : public BLEServerCallbacks, public BLECharacteristicCallbacks {
 public:
  static GloveBLEServer& getInstance() {
    static GloveBLEServer instance;
    return instance;
  }
  GloveBLEServer(const GloveBLEServer&) = delete;
  GloveBLEServer& operator=(const GloveBLEServer&) = delete;

  void loop() {
    if (device_connected_ && !last_device_connected_) {
      advertising_->stop();
    }
    if (!device_connected_ && last_device_connected_) {
      advertising_->start();
    }
    if (device_connected_) {
      setPatternSyncMessage(GloveApp::getInstance().next_motor_cycle_start, Glove::getInstance().getCurrentPattern());
      pattern_characteristic_->notify();
    }
    last_device_connected_ = device_connected_;
  }

 private:
  GloveBLEServer() : server_(nullptr), service_(nullptr), pattern_characteristic_(nullptr), mac_characteristic_(nullptr),
                     advertising_(nullptr), device_connected_(false), last_device_connected_(false) {
    BLEDevice::init("Vibrating Glove (Server)");
    server_ = BLEDevice::createServer();
    server_->setCallbacks(this);
    service_ = server_->createService(SERVICE_UUID);
    pattern_characteristic_ = service_->createCharacteristic(
      PATTERN_SYNC_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
      
    mac_characteristic_ = service_->createCharacteristic(
      ESPNOW_MAC_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    mac_characteristic_->setCallbacks(this);

    uint8_t mac[6];
    WiFi.macAddress(mac);
    mac_characteristic_->setValue(mac, 6);

    setPatternSyncMessage(GloveApp::getInstance().next_motor_cycle_start, Glove::getInstance().getCurrentPattern());
    service_->start();

    advertising_ = BLEDevice::getAdvertising();
    advertising_->addServiceUUID(SERVICE_UUID);
    advertising_->setScanResponse(true);
    advertising_->setMinPreferred(BLE_ADV_MIN_PREF_INTERVAL);
    advertising_->setMaxPreferred(BLE_ADV_MAX_PREF_INTERVAL);
    advertising_->start();
  }


  void onConnect(BLEServer*) override {
    device_connected_ = true;
    Serial.println("Client connected");
  }
  
  void onDisconnect(BLEServer*) override {
    device_connected_ = false;
    Serial.println("Client disconnected");
  }

  void onWrite(BLECharacteristic *pCharacteristic) override {
    if (pCharacteristic->getUUID().equals(BLEUUID(ESPNOW_MAC_CHARACTERISTIC_UUID))) {
      String rxValue = pCharacteristic->getValue();
      if (rxValue.length() == 6) {
        const uint8_t* mac = (const uint8_t*)rxValue.c_str();
        GloveEspNow::getInstance().addPeer(mac);
        Serial.printf("Received Client MAC Address via BLE: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      }
    }
  }

  void setPatternSyncMessage(TickType_t next_cycle, int pattern) {
    PatternSyncMessage message = { next_cycle, pattern };
    pattern_characteristic_->setValue((uint8_t*)&message, sizeof(message));
  }

  BLEServer* server_;
  BLEService* service_;
  BLECharacteristic* pattern_characteristic_;
  BLECharacteristic* mac_characteristic_;
  BLEAdvertising* advertising_;
  volatile bool device_connected_;
  volatile bool last_device_connected_;
};

class GloveBLEClient : public BLEClientCallbacks {
 public:
  static GloveBLEClient& getInstance() {
    static GloveBLEClient instance;
    return instance;
  }
  GloveBLEClient(const GloveBLEClient&) = delete;
  GloveBLEClient& operator=(const GloveBLEClient&) = delete;

  void loop() {
    if (!(connected_to_server_ || connectToServer())) return;
  }

 private:
  GloveBLEClient() : scan_(nullptr), client_(nullptr), connected_to_server_(false) {
    BLEDevice::init("Vibrating Glove (Client)");
    scan_ = BLEDevice::getScan();
    scan_->setActiveScan(true);
    scan_->setInterval(BLE_SCAN_INTERVAL);
    scan_->setWindow(BLE_SCAN_WINDOW);
    client_ = BLEDevice::createClient();
  }


  void onConnect(BLEClient* pclient) override {
    Serial.println("Connected to server");
  }

  void onDisconnect(BLEClient* pclient) override {
    connected_to_server_ = false;
    Serial.println("Disconnected from server");
  }

  static void notifyCallback(BLERemoteCharacteristic* rc, uint8_t* data, size_t len, bool is_notify) {
    static TickType_t last_cycle_start_received = 0;
    PatternSyncMessage message;

    if (len != sizeof(message)) {
      Serial.printf("Wrong data length: %u, expected %d\n", len, sizeof(message));
      return;
    }

    memcpy(&message, data, len);
    TickType_t next_cycle_start = message.next_cycle_start;
    Glove::getInstance().setCurrentPattern(message.pattern);

    if (last_cycle_start_received != next_cycle_start) {
      last_cycle_start_received = next_cycle_start;
      GloveApp::getInstance().next_motor_cycle_start = next_cycle_start + GloveApp::getInstance().current_tick_drift;
      Serial.printf(
        "New pattern (%d). Next motor cycle at %u\n",
        Glove::getInstance().getCurrentPattern(), GloveApp::getInstance().next_motor_cycle_start);
        
      GloveApp::getInstance().notifyMotorTask();
    }
  }

  using NotifyCb = void (*)(BLERemoteCharacteristic*, uint8_t*, size_t, bool);

  bool bindCharacteristic(BLERemoteService* remote_service, const char* char_uuid, NotifyCb cb = nullptr) {
    BLERemoteCharacteristic* rx_char = remote_service->getCharacteristic(char_uuid);
    if (rx_char == nullptr) {
      Serial.printf("Characteristic %s not found on server\n", char_uuid);
      return false;
    }
    
    if (cb != nullptr) {
      if (!rx_char->canNotify()) {
        Serial.printf("Characteristic %s does not support notify capabilities\n", char_uuid);
        return false;
      }
      rx_char->registerForNotify(cb);
    }
    return true;
  }

  bool connectToServer() {
    if (connected_to_server_) return true;

    BLEScanResults* found_devices = scan_->start(BLE_SCAN_TIME_S, false);
    Serial.print("Devices found: ");
    Serial.println(found_devices->getCount());
    for (int dev_id = 0; dev_id < found_devices->getCount(); ++dev_id) {
      auto remote_device = found_devices->getDevice(dev_id);
      BLEUUID service_uuid = remote_device.getServiceUUID();
      if (service_uuid.equals(BLEUUID(SERVICE_UUID))) {
        Serial.println("Found a device! Validating...");
        client_->setClientCallbacks(this);
        client_->connect(&remote_device);
        BLERemoteService* remote_service = client_->getService(SERVICE_UUID);
        if (remote_service == nullptr) {
          Serial.println("We don't have the right service?");
          client_->disconnect();
          continue;
        }

        if (!bindCharacteristic(remote_service, PATTERN_SYNC_CHARACTERISTIC_UUID, notifyCallback)) {
          client_->disconnect();
          continue;
        }

        BLERemoteCharacteristic* rx_mac_char = remote_service->getCharacteristic(ESPNOW_MAC_CHARACTERISTIC_UUID);
        if (rx_mac_char == nullptr) {
          Serial.println("ESPNOW MAC Characteristic not found on server");
          client_->disconnect();
          continue;
        }

        if (rx_mac_char->canRead()) {
          String value = rx_mac_char->readValue();
          if (value.length() == 6) {
            const uint8_t* mac = (const uint8_t*)value.c_str();
            GloveEspNow::getInstance().addPeer(mac);
            Serial.printf("Read Server MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
          }
        }

        if (rx_mac_char->canWrite()) {
          uint8_t mac[6];
          WiFi.macAddress(mac);
          rx_mac_char->writeValue(mac, 6);
          Serial.println("Sent Client MAC Address to Server.");
        }

        connected_to_server_ = true;
        break;
      }
    }
    scan_->clearResults();
    return connected_to_server_;
  }

  BLEScan* scan_;
  BLEClient* client_;
  volatile bool connected_to_server_;
};

