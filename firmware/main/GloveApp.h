#pragma once
#include <Arduino.h>
#include "config.h"

class GloveApp {
 public:
  static GloveApp& getInstance() {
    static GloveApp instance;
    return instance;
  }
  GloveApp(const GloveApp&) = delete;
  GloveApp& operator=(const GloveApp&) = delete;

  volatile uint32_t sync_time = 0;
  volatile TickType_t next_motor_cycle_start = 0;
  volatile int32_t current_tick_drift = 0;

  static void IRAM_ATTR sync_isr();

  void notifyMotorTask() {
    if (motor_task_handle_ != nullptr) {
      xTaskNotifyGive(motor_task_handle_);
    }
  }

  void start(bool is_server);

 private:
  GloveApp() = default;

  TaskHandle_t ble_task_handle_ = nullptr;
  TaskHandle_t motor_task_handle_ = nullptr;

  static void ble_server_task(void* parameter);
  static void ble_client_task(void* parameter);
  static void motor_server_task(void* parameter);
  static void motor_client_task(void* parameter);
};
