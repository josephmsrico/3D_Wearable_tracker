#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>
#include "packets.h"

// ---------- Settings ----------
#define RGB_PIN 21
#define NUM_PIXELS 1
#define CONNECTION_TIMEOUT_MS 4000
#define STATUS_CHECK_INTERVAL_MS 3000
#define REPORT_MS 1000

#ifndef ANCHOR_ID
#define ANCHOR_ID 1          // set per board in platformio.ini
#endif
uint8_t HUB_MAC[6] = {0x68, 0xEE, 0x8F, 0xE3, 0x5F, 0xF1};   // Hub's AP MAC — must match the Hub's boot message

Adafruit_NeoPixel pixel(NUM_PIXELS, RGB_PIN, NEO_RGB + NEO_KHZ800);

// ---------- Shared between BLE task and loop() ----------
struct Slot { long rssiSum; uint8_t count; TrackerStatus status; };
Slot slots[8];
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long lastSeenMs = 0;
volatile bool everSeen = false;

unsigned long lastStatusCheck = 0;
unsigned long lastReport = 0;

// Did the Hub actually receive our messages? (filled in by ESP-NOW)
volatile uint32_t deliveredOk = 0, deliveredFail = 0;

// ESP-NOW calls this after every send, telling us if the Hub acknowledged it
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) deliveredOk++;
  else                                deliveredFail++;
}

// Recognises Trackers by their magic byte
class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *dev) override {
    TrackerStatus s;
    if (!parseTrackerAdvert(dev->getManufacturerData(), s)) return;
    if (s.trackerId >= 8) return;

    lastSeenMs = millis();
    everSeen = true;

    portENTER_CRITICAL(&mux);
    if (slots[s.trackerId].count < 255) {
      slots[s.trackerId].rssiSum += dev->getRSSI();
      slots[s.trackerId].count++;
    }
    slots[s.trackerId].status = s;
    portEXIT_CRITICAL(&mux);
  }
};

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

// Broadcast a slow "I'm here" beacon so Trackers know they're in range
void startNodeBeacon(uint8_t type, uint8_t id) {
  NodeBeacon b = { type, id };
  uint8_t mfr[2 + sizeof(b)];
  mfr[0] = COMPANY_ID & 0xFF;
  mfr[1] = COMPANY_ID >> 8;
  memcpy(mfr + 2, &b, sizeof(b));

  NimBLEAdvertisementData d;
  d.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  d.setCompleteServices(NimBLEUUID(NODE_SERVICE_UUID));
  d.setManufacturerData(std::string((char*)mfr, sizeof(mfr)));

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setAdvertisementData(d);
  adv->setMinInterval(400);   // 250 ms
  adv->setMaxInterval(480);   // 300 ms
  adv->start();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pixel.begin();
  pixel.setBrightness(50);

  // Blink white ANCHOR_ID times so you can identify this Anchor by eye
  for (int i = 0; i < ANCHOR_ID; i++) {
    setLed(255, 255, 255); delay(250);
    setLed(0, 0, 0);       delay(250);
  }

  setLed(0, 0, 255);   // blue = booting

  // ESP-NOW on the Hub's channel
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(true);                              // makes the channel change stick
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.printf("\n=== ANCHOR %d === MAC %s\n", ANCHOR_ID, WiFi.macAddress().c_str());
  Serial.printf("Sending to Hub %02X:%02X:%02X:%02X:%02X:%02X on channel %d\n",
                HUB_MAC[0], HUB_MAC[1], HUB_MAC[2], HUB_MAC[3], HUB_MAC[4], HUB_MAC[5], WIFI_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    setLed(255, 0, 255);   // purple = radio fault
    return;
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, HUB_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Add peer failed");
    setLed(255, 0, 255);
  }

  // BLE scanning for Trackers
  char name[12];
  snprintf(name, sizeof(name), "Anchor%02d", ANCHOR_ID);
  NimBLEDevice::init(name);
  NimBLEScan *pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new ScanCallbacks(), true);
  pScan->setDuplicateFilter(false);
  pScan->setActiveScan(false);
  pScan->setInterval(100);
  pScan->setWindow(50);
  pScan->start(0, false);

  // Tell Trackers this Anchor is here
  startNodeBeacon(NODE_ANCHOR, ANCHOR_ID);

  Serial.println("Scanning continuously...");
}

void loop() {
  // ---- Send averaged RSSI + status to the Hub every second ----
  if (millis() - lastReport >= REPORT_MS) {
    lastReport = millis();

    for (uint8_t id = 0; id < 8; id++) {
      AnchorReport r;
      bool have = false;

      portENTER_CRITICAL(&mux);
      if (slots[id].count > 0) {
        r.anchorId = ANCHOR_ID;
        r.rssi     = slots[id].rssiSum / slots[id].count;
        r.samples  = slots[id].count;
        r.status   = slots[id].status;
        slots[id].rssiSum = 0;
        slots[id].count   = 0;
        have = true;
      }
      portEXIT_CRITICAL(&mux);

      if (have) {
        esp_err_t err = esp_now_send(HUB_MAC, (uint8_t*)&r, sizeof(r));
        Serial.printf("T%d rssi %d (%d samples) -> %s\n",
                      id, r.rssi, r.samples, err == ESP_OK ? "queued" : "send error");
      }
    }
  }

  // ---- Status check: Tracker heard? Hub receiving? ----
  if (millis() - lastStatusCheck >= STATUS_CHECK_INTERVAL_MS) {
    lastStatusCheck = millis();
    bool trackerHeard = everSeen && (millis() - lastSeenMs < CONNECTION_TIMEOUT_MS);
    uint32_t ok = deliveredOk, fail = deliveredFail;
    deliveredOk = 0; deliveredFail = 0;
    bool hubOk = ok > 0;

    if (!trackerHeard) setLed(255, 0, 0);       // red    = can't hear the Tracker
    else if (!hubOk)   setLed(255, 160, 0);     // orange = hears Tracker, but Hub isn't receiving
    else               setLed(0, 255, 0);       // green  = all good

    Serial.printf("[%lu] Tracker: %s | Hub delivery: %lu ok, %lu failed%s\n",
                  millis(), trackerHeard ? "HEARD" : "NOT HEARD",
                  (unsigned long)ok, (unsigned long)fail,
                  (trackerHeard && !hubOk) ? "  <-- check HUB_MAC / channel" : "");
  }
}