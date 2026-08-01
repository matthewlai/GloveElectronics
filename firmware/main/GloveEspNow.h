#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

class GloveEspNow {
 public:
  static GloveEspNow& getInstance() {
    static GloveEspNow instance;
    return instance;
  }
  GloveEspNow(const GloveEspNow&) = delete;
  GloveEspNow& operator=(const GloveEspNow&) = delete;

  void init(bool is_server);
  void addPeer(const uint8_t* mac);
  void sendSync();
  void wakeUp();
  void sleep();
  void loop();

 private:
  GloveEspNow() = default;
  static void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len);

  uint8_t peer_mac_address[6] = {0};
  bool peer_added = false;
  bool is_server_mode = false;
  esp_now_peer_info_t peerInfo;
  uint32_t last_sync_time = 0;
  bool is_wifi_awake = true;
  uint8_t retry_count = 0;
};
