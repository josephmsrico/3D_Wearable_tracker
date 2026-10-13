#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <NimBLEDevice.h>
#include "packets.h"

// ---------- Network ----------
const char* AP_SSID = "TrackerHub";
const char* AP_PASS = "firetrack123";

// ---------- Receivers: index 0 = Hub, 1..3 = Anchor IDs (metres) ----------
#define MAX_RX 4
#define MAX_TRACKERS 8
const float RX_X[MAX_RX] = { 0.0, 10.0,  0.0, 10.0 };   // Hub, A1, A2, A3
const float RX_Y[MAX_RX] = { 0.0,  0.0,  8.0,  8.0 };

// Calibration (placeholders until phase 3) 
float TX_POWER_AT_1M = -59.0;
float PATH_LOSS_EXP  = 2.5;

//  Fusion settings 
const float ROOM_ROTATION_DEG = 0.0;     // compass heading of the room's +Y axis
const float FIX_GAIN = 0.3;              // how strongly an RSSI fix corrects position
const unsigned long DR_MAX_MS = 30000;   // dead-reckon at most 30 s after the last fix
const float MAX_PDR_JUMP_M = 5.0;        // ignore step jumps bigger than this (Tracker restarted)
const float FLOOR_HEIGHT_M = 2.5;

const unsigned long READING_MAX_AGE_MS = 3000;
const unsigned long SOLVE_MS = 1000;

// Shared data (BLE task + ESP-NOW task + loop) 
struct Reading { float rssi; unsigned long t; };
Reading readings[MAX_TRACKERS][MAX_RX];
TrackerStatus statusOf[MAX_TRACKERS];
long hubSum[MAX_TRACKERS];
uint8_t hubCount[MAX_TRACKERS];
unsigned long lastDirectMs[MAX_TRACKERS];
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//  Position state (only touched in loop) 
float posX[MAX_TRACKERS], posY[MAX_TRACKERS];
bool hasPos[MAX_TRACKERS];
unsigned long lastFixMs[MAX_TRACKERS];
int16_t lastSeq[MAX_TRACKERS];                 // -1 = nothing yet
float lastPdrE[MAX_TRACKERS], lastPdrN[MAX_TRACKERS];

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
unsigned long lastSolve = 0;

//  Receiving data 

class HubScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    TrackerStatus s;
    if (!parseTrackerAdvert(dev->getManufacturerData(), s)) return;
    if (s.trackerId >= MAX_TRACKERS) return;

    portENTER_CRITICAL(&mux);
    if (hubCount[s.trackerId] < 255) {
      hubSum[s.trackerId] += dev->getRSSI();
      hubCount[s.trackerId]++;
    }
    statusOf[s.trackerId] = s;
    lastDirectMs[s.trackerId] = millis();
    portEXIT_CRITICAL(&mux);
  }
};

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != sizeof(AnchorReport)) return;
  AnchorReport r;
  memcpy(&r, data, sizeof(r));
  if (r.status.magic != TRACKER_MAGIC) return;
  uint8_t id = r.status.trackerId;
  if (r.anchorId == 0 || r.anchorId >= MAX_RX || id >= MAX_TRACKERS) return;

  portENTER_CRITICAL(&mux);
  readings[id][r.anchorId].rssi = r.rssi;
  readings[id][r.anchorId].t    = millis();
  statusOf[id] = r.status;
  portEXIT_CRITICAL(&mux);
}

// RSSI calculations

float rssiToMetres(float rssi) {
  float d = powf(10.0f, (TX_POWER_AT_1M - rssi) / (10.0f * PATH_LOSS_EXP));
  return constrain(d, 0.3f, 50.0f);
}
// 2D Trilateration //
bool trilaterate(const float* xs, const float* ys, const float* ds, int n,
                 float& x, float& y) {
  int k = n - 1;
  float a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0;
  for (int i = 0; i < k; i++) {
    float ax = 2 * (xs[i] - xs[k]);
    float ay = 2 * (ys[i] - ys[k]);
    float b  = ds[k]*ds[k] - ds[i]*ds[i]
             + xs[i]*xs[i] - xs[k]*xs[k]
             + ys[i]*ys[i] - ys[k]*ys[k];
    a11 += ax * ax;  a12 += ax * ay;  a22 += ay * ay;
    b1  += ax * b;   b2  += ay * b;
  }
  float det = a11 * a22 - a12 * a12;
  if (fabsf(det) < 1e-3f) return false;
  x = (a22 * b1 - a12 * b2) / det;
  y = (a11 * b2 - a12 * b1) / det;
  return true;
}

// ================= Solve + send =================

void solveAndSend() {
  unsigned long now = millis();

  portENTER_CRITICAL(&mux);
  for (int id = 0; id < MAX_TRACKERS; id++) {
    if (hubCount[id] > 0) {
      readings[id][0].rssi = (float)hubSum[id] / hubCount[id];
      readings[id][0].t    = now;
      hubSum[id] = 0;
      hubCount[id] = 0;
    }
  }
  portEXIT_CRITICAL(&mux);

  for (int id = 0; id < MAX_TRACKERS; id++) {
    Reading r[MAX_RX];
    TrackerStatus s;
    unsigned long lastDirect;
    portENTER_CRITICAL(&mux);
    memcpy(r, readings[id], sizeof(r));
    s = statusOf[id];
    lastDirect = lastDirectMs[id];
    portEXIT_CRITICAL(&mux);
    if (s.magic != TRACKER_MAGIC) continue;          // never heard this Tracker
    bool direct = lastDirect != 0 && now - lastDirect < READING_MAX_AGE_MS;

    // ---- Fresh receivers ----
    float xs[MAX_RX], ys[MAX_RX], ds[MAX_RX];
    int n = 0, nearest = -1;
    float nearestD = 1e9;
    for (int rx = 0; rx < MAX_RX; rx++) {
      if (r[rx].t == 0 || now - r[rx].t > READING_MAX_AGE_MS) continue;
      float d = rssiToMetres(r[rx].rssi);
      xs[n] = RX_X[rx];  ys[n] = RX_Y[rx];  ds[n] = d;  n++;
      Serial.printf("T%d  rx%d  rssi %.0f  ~%.1f m\n", id, rx, r[rx].rssi, d);
      if (d < nearestD) { nearestD = d; nearest = rx; }
    }
    if (n == 0) continue;                            // nobody hears this Tracker

    // ---- 1. Dead reckoning: move by the steps walked since the last update ----
    float e  = s.pdrXdm / 10.0f, nn = s.pdrYdm / 10.0f;
    bool newData = true;
    if (lastSeq[id] >= 0) {
      int8_t d = (int8_t)(uint8_t)(s.seq - (uint8_t)lastSeq[id]);
      newData = (d > 0) || (d < -5);                 // newer, or Tracker restarted
    }
    if (newData) {
      if (lastSeq[id] >= 0 && hasPos[id]) {
        float de = e - lastPdrE[id], dn = nn - lastPdrN[id];
        if (fabsf(de) < MAX_PDR_JUMP_M && fabsf(dn) < MAX_PDR_JUMP_M) {
          float th = ROOM_ROTATION_DEG * DEG_TO_RAD;  // compass frame -> room frame
          posX[id] += de * cosf(th) - dn * sinf(th);
          posY[id] += de * sinf(th) + dn * cosf(th);
        }
      }
      lastSeq[id] = s.seq;  lastPdrE[id] = e;  lastPdrN[id] = nn;
    }

    // ---- 2. RSSI fix corrects the dead-reckoned position ----
    const char* mode;
    float outX, outY, range = 0, fx, fy;
    if (n >= 3 && trilaterate(xs, ys, ds, n, fx, fy)) {
      if (!hasPos[id]) { posX[id] = fx; posY[id] = fy; hasPos[id] = true; }
      else {
        posX[id] += FIX_GAIN * (fx - posX[id]);
        posY[id] += FIX_GAIN * (fy - posY[id]);
      }
      lastFixMs[id] = now;
      mode = "fix";  outX = posX[id];  outY = posY[id];
    } else if (hasPos[id] && now - lastFixMs[id] < DR_MAX_MS) {
      mode = "dr";   outX = posX[id];  outY = posY[id];  range = nearestD;
    } else {
      mode = "range"; outX = RX_X[nearest]; outY = RX_Y[nearest]; range = nearestD;
    }

    // ---- 3. Send everything to the dashboard ----
    float alt = s.altDm / 10.0f;
    char buf[420];
    snprintf(buf, sizeof(buf),
      "{\"id\":\"T%d\",\"mode\":\"%s\",\"x\":%.2f,\"y\":%.2f,\"r\":%.2f,"
      "\"rx\":%d,\"direct\":%d,\"floor\":%d,\"alt\":%.1f,"
      "\"temp\":%.1f,\"press\":%.1f,\"hdg\":%u,\"steps\":%u,"
      "\"pdrX\":%.1f,\"pdrY\":%.1f,\"bat\":%u,"
      "\"moving\":%d,\"mandown\":%d,\"heat\":%d,\"fault\":%d,\"seq\":%u}",
      id, mode, outX, outY, range, n, direct ? 1 : 0,
      (int)lroundf(alt / FLOOR_HEIGHT_M), alt,
      s.tempC10 / 10.0f, s.pressHpa10 / 10.0f, s.heading, s.steps,
      e, nn, s.battery,
      (s.flags & FLAG_MOVING)  ? 1 : 0, (s.flags & FLAG_MANDOWN) ? 1 : 0,
      (s.flags & FLAG_HEAT)    ? 1 : 0, (s.flags & FLAG_FAULT)   ? 1 : 0, s.seq);
    ws.textAll(buf);
    Serial.println(buf);
  }
}

// ================= Beacon =================

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
  adv->setMinInterval(400);
  adv->setMaxInterval(480);
  adv->start();
}

// ================= Setup / loop =================

void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT)    Serial.printf("Dashboard #%u connected\n", client->id());
  if (type == WS_EVT_DISCONNECT) Serial.printf("Dashboard #%u left\n", client->id());
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== COMMAND HUB ===");

  for (int i = 0; i < MAX_TRACKERS; i++) lastSeq[i] = -1;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, WIFI_CHANNEL);
  Serial.print("Dashboard at: http://");
  Serial.println(WiFi.softAPIP());
  Serial.print("Hub AP MAC (copy into each Anchor's HUB_MAC): ");
  Serial.println(WiFi.softAPmacAddress());

  if (esp_now_init() != ESP_OK) Serial.println("ESP-NOW init failed — Anchors won't be heard");
  else esp_now_register_recv_cb(onDataRecv);

  if (!LittleFS.begin()) Serial.println("LittleFS mount failed — run uploadfs");
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.begin();

  NimBLEDevice::init("Hub");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new HubScanCallbacks(), true);
  scan->setDuplicateFilter(false);
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(30);
  scan->start(0, false);

  startNodeBeacon(NODE_HUB, 0);
  Serial.println("Hub running: scanning + listening for Anchors");
}

void loop() {
  if (millis() - lastSolve >= SOLVE_MS) {
    lastSolve = millis();
    solveAndSend();
  }
  ws.cleanupClients();
}