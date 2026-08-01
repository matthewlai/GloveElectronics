#include "GloveEspNow.h"
#include "GloveApp.h"
#include "config.h"
#include <esp_wifi.h>



typedef struct __attribute__((packed)) EspNowSyncRequest {
  TickType_t client_tx_ticks;
} EspNowSyncRequest;

typedef struct __attribute__((packed)) EspNowSyncResponse {
  TickType_t client_tx_ticks;
  TickType_t server_tx_ticks;
} EspNowSyncResponse;

void GloveEspNow::OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len) {
  if (getInstance().is_server_mode) {
    if (len != sizeof(EspNowSyncRequest)) return;
    EspNowSyncRequest req;
    memcpy(&req, incomingData, sizeof(req));
    
    EspNowSyncResponse resp;
    resp.client_tx_ticks = req.client_tx_ticks;
    resp.server_tx_ticks = xTaskGetTickCount();
    
    esp_now_send(getInstance().peer_mac_address, (uint8_t*)&resp, sizeof(resp));
    getInstance().last_sync_time = millis();
    GloveApp::getInstance().notifyMotorTask();
  } else {
    if (len != sizeof(EspNowSyncResponse)) return;
    EspNowSyncResponse resp;
    memcpy(&resp, incomingData, sizeof(resp));
    
    TickType_t client_rx_ticks = xTaskGetTickCount();
    int32_t rtt = (int32_t)(client_rx_ticks - resp.client_tx_ticks);
    TickType_t server_time_at_rx = resp.server_tx_ticks + (rtt / 2);
    
    int32_t tick_drift = (int32_t)(client_rx_ticks - server_time_at_rx);
    GloveApp::getInstance().current_tick_drift = tick_drift;
    
    Serial.printf("ESPNOW Sync received! RTT: %d ms, Drift: %d\n", pdTICKS_TO_MS(rtt), tick_drift);
    
    static TickType_t last_server_ticks = 0;
    static TickType_t last_client_ticks = 0;

    if (last_server_ticks != 0) {
      TickType_t server_interval = resp.server_tx_ticks - last_server_ticks;
      TickType_t client_interval = client_rx_ticks - last_client_ticks;
      if (server_interval > 0) {
        int32_t interval_drift = (int32_t)client_interval - (int32_t)server_interval;
        float drift_rate_ppm = ((float)interval_drift * 1000000.0f) / (float)server_interval;
        Serial.printf("Clock drift rate: %.2f ppm (%d ms drift over %u ms interval)\n", drift_rate_ppm, pdTICKS_TO_MS(interval_drift), pdTICKS_TO_MS(server_interval));
      }
    }

    last_server_ticks = resp.server_tx_ticks;
    last_client_ticks = client_rx_ticks;
    getInstance().last_sync_time = millis();
    getInstance().retry_count = 0;
    
    GloveEspNow::getInstance().sleep();
  }
}

void GloveEspNow::init(bool is_server) {
  is_server_mode = is_server;
  WiFi.mode(WIFI_STA);
  
  // Power up radio initially to handle BLE MAC exchange
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
}

void GloveEspNow::addPeer(const uint8_t* mac) {
  memcpy(peer_mac_address, mac, 6);
  memcpy(peerInfo.peer_addr, peer_mac_address, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  peer_added = true;
  // Set last_sync_time so that Server sends a sync immediately and client expects one immediately.
  last_sync_time = millis() - ESPNOW_SYNC_INTERVAL_MS; 
}

void GloveEspNow::sendSync() {
  if (!peer_added) return;
  EspNowSyncRequest req;
  req.client_tx_ticks = xTaskGetTickCount();
  esp_now_send(peer_mac_address, (uint8_t *) &req, sizeof(req));
}

void GloveEspNow::wakeUp() {
  if (is_wifi_awake) return;
  
  WiFi.mode(WIFI_STA);
  esp_now_init();
  
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
  
  if (peer_added) {
    esp_now_add_peer(&peerInfo);
  }
  
  is_wifi_awake = true;
}

void GloveEspNow::sleep() {
  if (!is_wifi_awake) return;
  
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
  
  is_wifi_awake = false;
}

void GloveEspNow::loop() {
  if (!peer_added) return;

  uint32_t now = millis();
  if (is_server_mode) {
    uint32_t time_since_last_sync = now - last_sync_time;
    if (time_since_last_sync >= (ESPNOW_SYNC_INTERVAL_MS - ESPNOW_WAKEUP_MARGIN_MS) && time_since_last_sync < (ESPNOW_SYNC_INTERVAL_MS + 5000)) {
      wakeUp();
    } else if (time_since_last_sync >= (ESPNOW_SYNC_INTERVAL_MS + 5000)) {
      last_sync_time = now;
      Serial.println("Server didn't see sync request. Sleeping.");
      sleep();
    } else if (time_since_last_sync >= 2 * ESPNOW_SYNC_RETRY_MS) {
      sleep();
    }
  } else {
    uint32_t time_since_last_sync = now - last_sync_time;
    if (time_since_last_sync >= (ESPNOW_SYNC_INTERVAL_MS - ESPNOW_WAKEUP_MARGIN_MS)) {
      wakeUp();
    }
    
    if (time_since_last_sync >= ESPNOW_SYNC_INTERVAL_MS) {
      static uint32_t last_ping_time = 0;
      if (now - last_ping_time > ESPNOW_SYNC_RETRY_MS) {
        if (retry_count < ESPNOW_MAX_RETRIES) {
          sendSync();
          last_ping_time = now;
          retry_count++;
        } else {
          last_sync_time = now;
          retry_count = 0;
          Serial.println("Warning: Missed ESPNOW sync response! Sleeping.");
          sleep();
        }
      }
    }
  }
}
