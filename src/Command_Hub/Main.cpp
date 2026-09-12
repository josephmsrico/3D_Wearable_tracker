#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

typedef struct AnchorPacket {
  unsigned long timestamp_ms;
  char mac_address[18];
  int rssi;
} AnchorPacket;

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  AnchorPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));

  Serial.print(packet.timestamp_ms);
  Serial.print(",");
  Serial.print(packet.mac_address);
  Serial.print(",");
  Serial.println(packet.rssi);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(500);

  WiFi.mode(WIFI_STA);
  Serial.print("Command Hub MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.println("timestamp_ms,mac_address,rssi");
  Serial.println("Command Hub ready, listening for anchor data...");
}

void loop() {
  delay(10);
}