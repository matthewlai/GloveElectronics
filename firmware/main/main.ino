// Select ESP32C3 Dev Module as the board, and enable "USB CDC On Boot" in the
// "Tools" menu.

#include <algorithm>
#include <utility>

#include "GloveApp.h"
#include "GloveBluetooth.h"
#include "GloveHardware.h"
#include "config.h"

void setup() {
  Serial.begin();
  delay(500);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DRV_EN_PIN, OUTPUT);
  pinMode(SYNC_SERVER_CLIENT_SELECT_PIN, INPUT_PULLUP);
  
  // Force evaluation of singletons now so hardware constructors run in main thread
  I2CMux::getInstance();
  Glove::getInstance();

  bool is_ble_server = digitalRead(SYNC_SERVER_CLIENT_SELECT_PIN) == HIGH;
  GloveApp::getInstance().start(is_ble_server);
}

// Low-priority housekeeping
void loop() {
  if (!Glove::getInstance().isBatteryOK()) {
    Serial.printf("Low battery (%fv). Shutting down\n",
                  Glove::getInstance().getBatteryVoltage());
    Glove::getInstance().shutdown();
  }

  if (millis() > (TOTAL_RUN_TIME_S * 1000)) {
    Serial.println("We are done! Shutting down now");
    Glove::getInstance().shutdown(false);
  }
}
