#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include "packets.h"
#include "Positioning.h"

// ---------- Network ----------
const char* AP_SSID = "TrackerHub";
const char* AP_PASS = "firetrack123";

// ---------- Receivers: index 0 = Hub, 1..3 = Anchor IDs (metres) ----------
// These are only the first-boot defaults. Set the real positions from the dashboard's
// Setup panel: they're saved in the Hub's flash and survive reboots and reflashing.
#define MAX_RX 4
#define MAX_TRACKERS 8
float RX_X[MAX_RX] = { 0.0, 10.0,  0.0, 10.0 };   // Hub, A1, A2, A3
float RX_Y[MAX_RX] = { 0.0,  0.0,  8.0,  8.0 };

// Calibration defaults — also settable from the dashboard, `cal`, or `calhere`
float TX_POWER_AT_1M = -59.0;
float PATH_LOSS_EXP  = 2.5;

// ---------- Fusion settings ----------
const float ROOM_ROTATION_DEG = 0.0;     // compass heading of the room's +Y axis
const float FIX_GAIN = 0.5;              // how strongly an RSSI fix corrects position (higher = faster)
const float NEAR_SNAP_M = 1.5;           // closer than this to a receiver = pull the fix toward it
const float FIX_MARGIN_M = 1.0;          // fixes are kept inside the receivers' box + this margin
const unsigned long DR_MAX_MS = 30000;   // dead-reckon at most 30 s after the last fix
const float MAX_PDR_JUMP_M = 5.0;        // ignore step jumps bigger than this (Tracker restarted)
const float FLOOR_HEIGHT_M = 3.0;        // storey height (3 m so small baro drift doesn't flip the floor)

const unsigned long READING_MAX_AGE_MS = 3000;
const unsigned long SOLVE_MS = 1000;

// ---------- Shared data (BLE task + ESP-NOW task + loop) ----------
struct Reading { float rssi; unsigned long t; };
Reading readings[MAX_TRACKERS][MAX_RX];
TrackerStatus statusOf[MAX_TRACKERS];
long hubSum[MAX_TRACKERS];
uint8_t hubCount[MAX_TRACKERS];
unsigned long lastDirectMs[MAX_TRACKERS];
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ---------- Position state (only touched in loop) ----------
float posX[MAX_TRACKERS], posY[MAX_TRACKERS];
bool hasPos[MAX_TRACKERS];
unsigned long lastFixMs[MAX_TRACKERS];
int16_t lastSeq[MAX_TRACKERS];                 // -1 = nothing yet
float lastPdrE[MAX_TRACKERS], lastPdrN[MAX_TRACKERS];

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences prefs;
unsigned long lastSolve = 0;

// ---------- Commands arriving from the dashboard (handled in loop, not the web task) ----------
volatile bool wsCmdReady = false;
char wsCmd[160];

// ---------- Multi-node test logging ----------
bool  logOn  = false;
float truthX = NAN, truthY = NAN;   // where the Tracker really is (typed in during tests)

// ---------- Self-calibration: stand at a known spot and type `calhere x y` ----------
const unsigned long CAL_SETTLE_MS  = 3000;    // ignore readings from before you arrived
const unsigned long CAL_COLLECT_MS = 10000;   // then listen for 10 s
unsigned long calFrom = 0, calUntil = 0;      // calUntil = 0 means not collecting
float calX = 0, calY = 0;
double cSx = 0, cSy = 0, cSxx = 0, cSxy = 0, cSyy = 0;   // running sums for the straight-line fit
long   cN = 0;

// ================= Saved settings (flash) =================

void loadConfig() {
  prefs.begin("hub", false);
  if (prefs.getBytesLength("rxx") == sizeof(RX_X)) prefs.getBytes("rxx", RX_X, sizeof(RX_X));
  if (prefs.getBytesLength("rxy") == sizeof(RX_Y)) prefs.getBytes("rxy", RX_Y, sizeof(RX_Y));
  TX_POWER_AT_1M = prefs.getFloat("tx", TX_POWER_AT_1M);
  PATH_LOSS_EXP  = prefs.getFloat("n",  PATH_LOSS_EXP);
  prefs.end();
}

void saveConfig() {
  prefs.begin("hub", false);
  prefs.putBytes("rxx", RX_X, sizeof(RX_X));
  prefs.putBytes("rxy", RX_Y, sizeof(RX_Y));
  prefs.putFloat("tx", TX_POWER_AT_1M);
  prefs.putFloat("n",  PATH_LOSS_EXP);
  prefs.end();
}

// Tells the dashboard where the receivers are and the current calibration
void sendConfig(AsyncWebSocketClient* client = nullptr) {
  char buf[220];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"config\",\"rx\":[[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f]],"
    "\"tx\":%.1f,\"n\":%.2f}",
    RX_X[0], RX_Y[0], RX_X[1], RX_Y[1], RX_X[2], RX_Y[2], RX_X[3], RX_Y[3],
    TX_POWER_AT_1M, PATH_LOSS_EXP);
  if (client) client->text(buf);
  else        ws.textAll(buf);
}

// Receivers moved: old positions no longer mean anything, start fresh
void resetPositions() {
  for (int i = 0; i < MAX_TRACKERS; i++) hasPos[i] = false;
}

// ================= Receiving data =================

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

// ================= RSSI -> distance =================

float rssiToMetres(float rssi) {
  float d = powf(10.0f, (TX_POWER_AT_1M - rssi) / (10.0f * PATH_LOSS_EXP));
  return constrain(d, 0.3f, 50.0f);
}

// (trilaterate() lives in Positioning.cpp)

// ================= Solve + send =================

void solveAndSend() {
  unsigned long now = millis();

  // Turn the Hub's own BLE samples from the last second into one averaged reading
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

    // ---- Fresh receivers -> positions + distances ----
    float xs[MAX_RX], ys[MAX_RX], ds[MAX_RX];
    float rxDist[MAX_RX];                            // each receiver's distance for the map (-1 = none)
    for (int i = 0; i < MAX_RX; i++) rxDist[i] = -1;
    int n = 0, nearest = -1;
    float nearestD = 1e9;
    for (int rx = 0; rx < MAX_RX; rx++) {
      if (r[rx].t == 0 || now - r[rx].t > READING_MAX_AGE_MS) continue;
      float d = rssiToMetres(r[rx].rssi);
      rxDist[rx] = d;

      // Self-calibration: we know where you're standing, so we know the TRUE distance
      if (calUntil && now >= calFrom) {
        float trueD = fmaxf(hypotf(calX - RX_X[rx], calY - RX_Y[rx]), 0.5f);
        double xv = 10.0 * log10(trueD), yv = r[rx].rssi;   // model: rssi = TX_1M - n * xv
        cSx += xv; cSy += yv; cSxx += xv * xv; cSxy += xv * yv; cSyy += yv * yv; cN++;
      }

      xs[n] = RX_X[rx];  ys[n] = RX_Y[rx];  ds[n] = d;  n++;
      Serial.printf("T%d  rx%d  rssi %.0f  ~%.1f m\n", id, rx, r[rx].rssi, d);
      if (logOn) Serial.printf("RAW,%lu,%d,%d,%.1f,%.2f,%.2f,%.2f\n",
                               now, id, rx, r[rx].rssi, d, truthX, truthY);
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

    // ---- 2. RSSI fix (2D trilateration) corrects the dead-reckoned position ----
    const char* mode;
    float outX, outY, range = 0, fx = 0, fy = 0, rms = -1;
    if (n >= 3 && trilaterate(xs, ys, ds, n, fx, fy, rms)) {
      // Keep the fix inside the box around the receivers that heard it
      float bx0 = xs[0], bx1 = xs[0], by0 = ys[0], by1 = ys[0];
      for (int i = 1; i < n; i++) {
        bx0 = fminf(bx0, xs[i]); bx1 = fmaxf(bx1, xs[i]);
        by0 = fminf(by0, ys[i]); by1 = fmaxf(by1, ys[i]);
      }
      fx = constrain(fx, bx0 - FIX_MARGIN_M, bx1 + FIX_MARGIN_M);
      fy = constrain(fy, by0 - FIX_MARGIN_M, by1 + FIX_MARGIN_M);

      // Right next to a receiver? Its reading is far more reliable than the distant ones,
      // so pull the fix toward it: 0% at NEAR_SNAP_M, 100% at 0 m.
      if (nearestD < NEAR_SNAP_M) {
        float pull = 1.0f - nearestD / NEAR_SNAP_M;
        fx += pull * (RX_X[nearest] - fx);
        fy += pull * (RX_Y[nearest] - fy);
      }

      float gain = FIX_GAIN / (1.0f + rms / 3.0f);   // trustworthy fix pulls harder
      if (!hasPos[id]) { posX[id] = fx; posY[id] = fy; hasPos[id] = true; }
      else {
        posX[id] += gain * (fx - posX[id]);
        posY[id] += gain * (fy - posY[id]);
      }
      lastFixMs[id] = now;
      mode = "fix";  outX = posX[id];  outY = posY[id];
    } else if (hasPos[id] && now - lastFixMs[id] < DR_MAX_MS) {
      mode = "dr";   outX = posX[id];  outY = posY[id];  range = nearestD;
    } else {
      mode = "range"; outX = RX_X[nearest]; outY = RX_Y[nearest]; range = nearestD;
    }

    // ---- Test log line ----
    if (logOn) {
      float err    = hypotf(outX - truthX, outY - truthY);                  // what the dashboard shows
      float rawErr = (rms >= 0) ? hypotf(fx - truthX, fy - truthY) : NAN;   // RSSI alone
      Serial.printf("FIX,%lu,%d,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f\n",
                    now, id, mode, outX, outY, fx, fy, rms, n, truthX, truthY, err, rawErr);
    }

    // ---- 3. Send everything to the dashboard ----
    float alt = s.altDm / 10.0f;
    char buf[600];
    snprintf(buf, sizeof(buf),
      "{\"id\":\"T%d\",\"mode\":\"%s\",\"x\":%.2f,\"y\":%.2f,\"r\":%.2f,\"rms\":%.2f,"
      "\"fx\":%.2f,\"fy\":%.2f,\"d\":[%.2f,%.2f,%.2f,%.2f],"
      "\"rx\":%d,\"direct\":%d,\"floor\":%d,\"alt\":%.1f,"
      "\"temp\":%.1f,\"press\":%.1f,\"hdg\":%u,\"steps\":%u,"
      "\"pdrX\":%.1f,\"pdrY\":%.1f,\"bat\":%u,"
      "\"moving\":%d,\"mandown\":%d,\"heat\":%d,\"fault\":%d,\"seq\":%u}",
      id, mode, outX, outY, range, rms,
      fx, fy, rxDist[0], rxDist[1], rxDist[2], rxDist[3],
      n, direct ? 1 : 0,
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

// ================= Calibration + commands =================

void printSetup() {
  Serial.printf("CAL,%.1f,%.2f\n", TX_POWER_AT_1M, PATH_LOSS_EXP);
  for (int i = 0; i < MAX_RX; i++) Serial.printf("RX,%d,%.2f,%.2f\n", i, RX_X[i], RX_Y[i]);
}

// Fit rssi = TX_1M - n * 10*log10(distance) through all collected samples (straight-line fit)
void finishCalibration() {
  calUntil = 0;
  double den = cN * cSxx - cSx * cSx;
  if (cN < 6 || den / ((double)cN * cN) < 4.0) {
    Serial.printf("# calibration: %ld samples, distances too similar — add a spot near a different node\n", cN);
    return;
  }
  double slope = (cN * cSxy - cSx * cSy) / den;
  double tx    = (cSy - slope * cSx) / cN;
  double n     = -slope;
  double sse   = cSyy - tx * cSy - slope * cSxy;
  double rmse  = sqrt(fmax(sse, 0.0) / cN);              // how well the line fits, in dB

  Serial.printf("# calibration from %ld samples: TX_1M = %.1f dBm, n = %.2f, fit error %.1f dB\n",
                cN, tx, n, rmse);
  if (n < 1.5 || n > 5.0 || tx < -95 || tx > -35) {
    Serial.println("# values look wrong — NOT applied. Try another spot, or `calreset` and start again");
    return;
  }
  TX_POWER_AT_1M = tx;
  PATH_LOSS_EXP  = n;
  saveConfig();
  sendConfig();
  Serial.println("# applied and saved");
}

// One command from the serial monitor OR the dashboard
void handleCommand(const char* line) {
  float a, b, v[10];
  int got = sscanf(line, "setup %f %f %f %f %f %f %f %f %f %f",
                   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8], &v[9]);
  if (got == 10) {
    // From the dashboard: 4 receiver positions (x y each), then TX_1M and n
    if (v[8] < -100 || v[8] > -20 || v[9] < 1.0 || v[9] > 6.0) {
      Serial.println("# setup rejected: TX_1M must be -100..-20 and n 1..6");
      sendConfig();                                   // put the dashboard back to the real values
      return;
    }
    for (int i = 0; i < MAX_RX; i++) { RX_X[i] = v[2 * i]; RX_Y[i] = v[2 * i + 1]; }
    TX_POWER_AT_1M = v[8];
    PATH_LOSS_EXP  = v[9];
    resetPositions();
    saveConfig();
    sendConfig();
    Serial.println("# setup saved from dashboard");
    printSetup();
  } else if (sscanf(line, "truth %f %f", &a, &b) == 2) {
    truthX = a; truthY = b; Serial.printf("# truth %.2f %.2f\n", a, b);
  } else if (!strcmp(line, "truth off")) {
    truthX = truthY = NAN; Serial.println("# truth off");
  } else if (sscanf(line, "calhere %f %f", &a, &b) == 2) {
    calX = a; calY = b;
    calFrom  = millis() + CAL_SETTLE_MS;
    calUntil = calFrom + CAL_COLLECT_MS;
    Serial.printf("# calibrating at (%.1f, %.1f): stand still for %lu s...\n",
                  a, b, (CAL_SETTLE_MS + CAL_COLLECT_MS) / 1000);
  } else if (!strcmp(line, "calreset")) {
    cSx = cSy = cSxx = cSxy = cSyy = 0; cN = 0;
    Serial.println("# calibration samples cleared");
  } else if (sscanf(line, "cal %f %f", &a, &b) == 2) {
    TX_POWER_AT_1M = a; PATH_LOSS_EXP = b;
    saveConfig(); sendConfig(); printSetup();
  } else if (!strcmp(line, "log on")) {
    logOn = true; printSetup();
  } else if (!strcmp(line, "log off")) {
    logOn = false;
  } else {
    Serial.println("# commands: calhere <x> <y> | calreset | cal <tx> <n> | log on | log off | truth <x> <y> | truth off");
  }
}

void handleSerial() {
  static char line[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { if (len < sizeof(line) - 1) line[len++] = c; continue; }
    line[len] = 0; len = 0;
    handleCommand(line);
  }
}

// ================= Setup / loop =================

void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("Dashboard #%u connected\n", client->id());
    sendConfig(client);                               // tell the new page where the receivers are
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("Dashboard #%u left\n", client->id());
  } else if (type == WS_EVT_DATA) {
    // A command from the dashboard: copy it, then loop() handles it
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT
        && len < sizeof(wsCmd) && !wsCmdReady) {
      memcpy(wsCmd, data, len);
      wsCmd[len] = 0;
      wsCmdReady = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== COMMAND HUB ===");

  for (int i = 0; i < MAX_TRACKERS; i++) lastSeq[i] = -1;
  loadConfig();
  printSetup();

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
  Serial.println("# type 'log on' to start test logging");
}

void loop() {
  if (millis() - lastSolve >= SOLVE_MS) {
    lastSolve = millis();
    solveAndSend();
  }
  if (calUntil && millis() >= calUntil) finishCalibration();
  if (wsCmdReady) { handleCommand(wsCmd); wsCmdReady = false; }
  handleSerial();
  ws.cleanupClients();
}