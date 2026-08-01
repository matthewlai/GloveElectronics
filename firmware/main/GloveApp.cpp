#include "GloveApp.h"
#include "GloveHardware.h"
#include "GloveBluetooth.h"
#include "GloveEspNow.h"

void IRAM_ATTR GloveApp::sync_isr() {
  getInstance().sync_time = millis();
}

void GloveApp::ble_server_task(void* parameter) {
  TickType_t last_wake_time = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&last_wake_time, ble_update_frequency);
    GloveBLEServer::getInstance().loop();
    GloveEspNow::getInstance().loop();
  }
}

void GloveApp::ble_client_task(void* parameter) {
  TickType_t last_wake_time = xTaskGetTickCount();

  while (true) {
    GloveBLEClient::getInstance().loop();
    GloveEspNow::getInstance().loop();
    vTaskDelayUntil(&last_wake_time, ble_update_frequency / 2);
  }
}

void GloveApp::motor_server_task(void* parameter) {
  pinMode(SYNC_PIN, INPUT);
  attachInterrupt(SYNC_PIN, GloveApp::sync_isr, RISING);
  int cycle_number = 0;

  // Wait until the first explicit ESP-NOW sync request is received from the Client
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  TickType_t last_wake_time = xTaskGetTickCount();

  while (true) {
    Glove::getInstance().setCurrentPattern((cycle_number++ % CYCLES_PER_ROUND < ACTIVE_CYCLES_PER_ROUND) ? random(Glove::getNumPatterns()) : -1);
    GloveApp::getInstance().next_motor_cycle_start = last_wake_time + FULL_CYCLE_LENGTH;
    vTaskDelayUntil(&last_wake_time, FULL_CYCLE_LENGTH);
    Glove::getInstance().runMotors(false);
  }
}

void GloveApp::motor_client_task(void* parameter) {
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);

  while (true) {
    // Hang here until the BLE callback announces a state/time change
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t target = GloveApp::getInstance().next_motor_cycle_start;
    
    // Convert to signed 32-bit math safely to prevent 49-day underflow sleeps
    // If the sync response came late and we're already past the start time, 
    // this will skip the sleep and instantly fire to catch up!
    int32_t delay = (int32_t)(target - last_wake_time);
    
    if (delay > 0) {
      vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(delay));
    }
    
    Glove::getInstance().runMotors(true);
  }
}

void GloveApp::start(bool is_server) {
  GloveEspNow::getInstance().init(is_server);
  
  Serial.printf("Starting %s tasks: ", is_server ? "server" : "client");
  if (is_server) {
    xTaskCreate(GloveApp::motor_server_task, "Motor task", TASK_STACK_SIZE, NULL, 10, &GloveApp::getInstance().motor_task_handle_);
    xTaskCreate(GloveApp::ble_server_task, "BLE task", TASK_STACK_SIZE, NULL, 10, &GloveApp::getInstance().ble_task_handle_);
  } else {
    xTaskCreate(GloveApp::motor_client_task, "Motor task", TASK_STACK_SIZE, NULL, 10, &GloveApp::getInstance().motor_task_handle_);
    xTaskCreate(GloveApp::ble_client_task, "BLE task", TASK_STACK_SIZE, NULL, 10, &GloveApp::getInstance().ble_task_handle_);
  }
  Serial.println(GloveApp::getInstance().motor_task_handle_ && GloveApp::getInstance().ble_task_handle_ ? "SUCCESS" : "FAILURE");
}
