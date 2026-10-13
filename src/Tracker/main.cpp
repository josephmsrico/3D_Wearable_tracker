#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP3XX.h>
#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>
#include "packets.h"

// ---------- Pins & addresses ----------
#define SDA_PIN 17                 // silkscreen "GPIO6"
#define SCL_PIN 18                 // silkscreen "GPIO7"
#define RESET_BUTTON_PIN 4         // silkscreen "GPIO20", other leg to GND
#define I2C_CLOCK 50000UL
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define ICM_ADDR 0x69
#define BMP_ADDR 0x77
#define RGB_PIN 21
#define NUM_PIXELS 1

#ifndef TRACKER_ID
#define TRACKER_ID 1
#endif

// ---------- Timing ----------
const int8_t  TX_POWER_DBM = 3;                    // fixed: calibration depends on this
const unsigned long IMU_INTERVAL_MS     = 20;      // 50 Hz: fast enough to catch footsteps
const unsigned long BARO_INTERVAL_MS    = 500;     // 2 Hz: environment changes slowly
const unsigned long STATUS_INTERVAL_MS  = 1000;    // refresh broadcast data every 1 s
const unsigned long PRINT_INTERVAL_MS   = 200;     // serial output 5 Hz
const unsigned long DISPLAY_INTERVAL_MS = 1000;    // OLED redraw blocks the I2C bus ~0.2 s
const unsigned long PAGE_INTERVAL       = 5000;
const unsigned long HEARTBEAT_MS        = 1000;
const unsigned long HEARTBEAT_OFF_MS    = 40;
const unsigned long RESET_HOLD_MS       = 1000;
const unsigned long LINK_TIMEOUT_MS     = 6000;
const uint8_t MAX_ANCHORS = 3;

// Motion (ICM-20948): tune by experiment
const float ACCEL_THRESH = 0.6;               // m/s² away from gravity = movement
const float GYRO_THRESH  = 0.3;               // rad/s rotation = movement
const unsigned long MOVE_HOLD_MS = 2000;
const unsigned long MANDOWN_MS   = 30000;     // no movement this long = MAN DOWN
const float STEP_HIGH = 11.0;                 // filtered |accel| must rise above this...
const float STEP_LOW  = 9.0;                  // ...after first dropping below this
const unsigned long STEP_MIN_MS = 300;        // at most ~3 steps per second
const float STEP_LENGTH_M = 0.70;             // average stride: measure your own
const float HEADING_GYRO_WEIGHT = 0.99;       // trust gyro short-term, compass long-term

// Which sensor axis points the way the wearer walks. (0,1,0) = board's +Y axis.
const float FWD_X = 0.0, FWD_Y = 1.0, FWD_Z = 0.0;

// Magnetometer hard-iron offsets in µT (see calibration steps)
const float MAG_OFF_X = 0.0, MAG_OFF_Y = 0.0, MAG_OFF_Z = 0.0;

//   Status Environment (BMP390) 
const float HEAT_TEMP_C         = 50.0;       // warn above this temperature
const float HEAT_RISE_C_PER_MIN = 10.0;       // or if it rises this fast
const float FLOOR_HEIGHT_M      = 3.0;        // storey height for floor estimate

// Objects 
Adafruit_ICM20948 icm;
Adafruit_BMP3XX bmp;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_NeoPixel pixel(NUM_PIXELS, RGB_PIN, NEO_RGB + NEO_KHZ800);
NimBLEAdvertising *pAdvertising;
sensors_event_t accel, gyro, temp, mag;
static const NimBLEUUID NODE_UUID(NODE_SERVICE_UUID);

// ---------- State ----------
bool displayOK = false, bmpOK = false, sensorFault = false, showMotionPage = true;
uint8_t bmpFailCount = 0;
float baselinePa = 101325.0, relAlt = 0, tempC = 0, pressPa = 0;
float tempHist[7];  uint8_t tempCount = 0;  float tempRisePerMin = 0;
float aFilt = 9.81;  bool stepArmed = true;  uint16_t steps = 0;
float heading = 0;   bool headingInit = false;
float pdrE = 0, pdrN = 0;                    // metres walked east / north since boot
uint8_t seq = 0;
uint32_t lastLedColor = 0xFFFFFFFF;
unsigned long lastImu = 0, lastBaro = 0, lastStatus = 0, lastPrint = 0;
unsigned long lastDisplay = 0, lastPageSwitch = 0;
unsigned long lastMotionMs = 0, lastStepMs = 0, lastTempHist = 0;

// When each node's beacon was last heard (written by the BLE task)
volatile unsigned long lastHubMs = 0;
volatile unsigned long lastAnchorMs[MAX_ANCHORS + 1] = {0};

// ================= Small helpers =================

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t c = pixel.Color(r, g, b);
  if (c == lastLedColor) return;
  lastLedColor = c;
  pixel.setPixelColor(0, c);
  pixel.show();
}

bool seen(unsigned long t) { return t != 0 && millis() - t < LINK_TIMEOUT_MS; }

bool linkUp() {
  if (seen(lastHubMs)) return true;
  for (int a = 1; a <= MAX_ANCHORS; a++) if (seen(lastAnchorMs[a])) return true;
  return false;
}

bool isMoving()    { return millis() - lastMotionMs < MOVE_HOLD_MS; }
bool isManDown()   { return millis() - lastMotionMs >= MANDOWN_MS; }
bool isHeatAlert() { return tempC >= HEAT_TEMP_C || tempRisePerMin >= HEAT_RISE_C_PER_MIN; }
int  floorNumber() { return (int)lroundf(relAlt / FLOOR_HEIGHT_M); }
uint8_t readBatteryPercent() { return 100; }   // TODO: MAX17048 fuel gauge

const char* compassPoint(float h) {
  static const char* pts[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return pts[(int)((h + 22.5f) / 45.0f) % 8];
}

// Builds e.g. "T1 Link:HUB A1" or "T1 Link:NONE"
void buildLinkLine(char* out, size_t n) {
  int len = snprintf(out, n, "T%d Link:", TRACKER_ID);
  bool any = false;
  if (seen(lastHubMs)) { len += snprintf(out + len, n - len, "HUB "); any = true; }
  for (int a = 1; a <= MAX_ANCHORS; a++)
    if (seen(lastAnchorMs[a])) { len += snprintf(out + len, n - len, "A%d ", a); any = true; }
  if (!any) snprintf(out + len, n - len, "NONE");
}

void bootMsg(const char* msg) {
  Serial.println(msg);
  if (displayOK) { display.println(msg); display.display(); }
}

// ================= Restart button & faults =================

void restartTracker() {
  Serial.println("Button held — restarting...");
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));   // blue = restarting
  pixel.show();
  if (displayOK) {
    display.clearDisplay();
    display.setCursor(0, 20);
    display.setTextSize(2);
    display.println("Restarting");
    display.display();
  }
  while (digitalRead(RESET_BUTTON_PIN) == LOW) delay(10);   // wait for release
  delay(200);
  ESP.restart();
}

void checkResetButton() {
  static bool wasDown = false;
  static unsigned long downSince = 0;
  bool down = (digitalRead(RESET_BUTTON_PIN) == LOW);
  if (down && !wasDown) downSince = millis();
  if (down && millis() - downSince >= RESET_HOLD_MS) restartTracker();
  wasDown = down;
}

void haltWithError(const char* msg) {
  pixel.setPixelColor(0, pixel.Color(255, 0, 0));
  pixel.show();
  if (displayOK) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("FAULT:");
    display.println(msg);
    display.println();
    display.println("Check wiring, then");
    display.println("hold btn to restart");
    display.display();
  }
  unsigned long lastPrintMs = 0;
  while (true) {
    if (millis() - lastPrintMs >= 1000) { lastPrintMs = millis(); Serial.println(msg); }
    checkResetButton();
    delay(10);                      // fine: Bluetooth hasn't started yet
  }
}

// ================= BMP390: environment =================

bool captureBaseline() {
  float sum = 0;
  int good = 0;
  for (int i = 0; i < 40 && good < 20; i++) {
    bool ok = bmp.performReading() && bmp.pressure > 80000 && bmp.pressure < 110000;
    if (ok && i >= 5) { sum += bmp.pressure; good++; }
    delay(30);                      // fine: runs before Bluetooth starts
  }
  if (good < 10) return false;
  baselinePa = sum / good;
  return true;
}

void processBaro(unsigned long now) {
  bmpOK = bmp.performReading();
  if (!bmpOK) {
    if (++bmpFailCount >= 10) sensorFault = true;
    return;
  }
  bmpFailCount = 0;
  tempC   = bmp.temperature;
  pressPa = bmp.pressure;
  relAlt  = 44330.0f * (1.0f - powf(pressPa / baselinePa, 0.1903f));

  // Temperature trend: one sample every 10 s, rise measured over the last minute
  if (tempCount == 0 || now - lastTempHist >= 10000) {
    lastTempHist = now;
    if (tempCount < 7) tempHist[tempCount++] = tempC;
    else { memmove(tempHist, tempHist + 1, 6 * sizeof(float)); tempHist[6] = tempC; }
    if (tempCount >= 4)
      tempRisePerMin = (tempHist[tempCount - 1] - tempHist[0]) * 60.0f / ((tempCount - 1) * 10.0f);
  }
}

// ================= ICM-20948: motion, heading, steps =================

// Compass heading of the "forward" axis, works whatever way the board is tilted.
// Returns -1 if it can't be computed.
float computeMagHeading(float ax, float ay, float az) {
  // "Down" in sensor frame: the accelerometer reads +g pointing UP at rest
  float dx = -ax, dy = -ay, dz = -az;
  // The magnetometer chip inside the ICM-20948 has its Y and Z axes flipped
  // relative to the accelerometer, so line them up first.
  float mx =  (mag.magnetic.x - MAG_OFF_X);
  float my = -(mag.magnetic.y - MAG_OFF_Y);
  float mz = -(mag.magnetic.z - MAG_OFF_Z);
  // East = Down × Mag,  North = East × Down
  float ex = dy * mz - dz * my, ey = dz * mx - dx * mz, ez = dx * my - dy * mx;
  float nx = ey * dz - ez * dy, ny = ez * dx - ex * dz, nz = ex * dy - ey * dx;
  float eLen = sqrtf(ex * ex + ey * ey + ez * ez);
  float nLen = sqrtf(nx * nx + ny * ny + nz * nz);
  if (eLen < 1e-6f || nLen < 1e-6f) return -1;
  float fe = (FWD_X * ex + FWD_Y * ey + FWD_Z * ez) / eLen;
  float fn = (FWD_X * nx + FWD_Y * ny + FWD_Z * nz) / nLen;
  float h = atan2f(fe, fn) * RAD_TO_DEG;
  return h < 0 ? h + 360 : h;
}

// Gyro gives smooth short-term turning; compass stops long-term drift
void updateHeading(float ax, float ay, float az, float gx, float gy, float gz,
                   float aMag, float dt) {
  if (aMag < 1.0f) return;
  float magH = computeMagHeading(ax, ay, az);
  if (!headingInit) {
    if (magH >= 0) { heading = magH; headingInit = true; }
    return;
  }
  // Turn rate around the vertical axis (deg/s, clockwise positive)
  float yawRate = -(gx * ax + gy * ay + gz * az) / aMag * RAD_TO_DEG;
  heading += yawRate * dt;
  if (magH >= 0) {
    float diff = magH - heading;
    while (diff > 180)  diff -= 360;
    while (diff < -180) diff += 360;
    heading += (1.0f - HEADING_GYRO_WEIGHT) * diff;
  }
  while (heading < 0)    heading += 360;
  while (heading >= 360) heading -= 360;
}

void processImu(unsigned long now) {
  static unsigned long lastT = 0;
  float dt = lastT ? (now - lastT) / 1000.0f : 0;
  lastT = now;

  icm.getEvent(&accel, &gyro, &temp, &mag);
  float ax = accel.acceleration.x, ay = accel.acceleration.y, az = accel.acceleration.z;
  float gx = gyro.gyro.x, gy = gyro.gyro.y, gz = gyro.gyro.z;
  float aMag = sqrtf(ax * ax + ay * ay + az * az);
  float gMag = sqrtf(gx * gx + gy * gy + gz * gz);

  // 1. Movement (used for "moving" and MAN DOWN)
  if (fabsf(aMag - 9.81f) > ACCEL_THRESH || gMag > GYRO_THRESH) lastMotionMs = now;

  // 2. Heading
  updateHeading(ax, ay, az, gx, gy, gz, aMag, dt);

  // 3. Steps: each footstep makes |accel| dip then peak
  aFilt = 0.7f * aFilt + 0.3f * aMag;                 // light smoothing
  if (aFilt < STEP_LOW) stepArmed = true;
  if (stepArmed && aFilt > STEP_HIGH && now - lastStepMs > STEP_MIN_MS) {
    stepArmed = false;
    lastStepMs = now;
    steps++;
    // 4. Dead reckoning: one stride in the current heading direction
    float h = heading * DEG_TO_RAD;
    pdrE += STEP_LENGTH_M * sinf(h);
    pdrN += STEP_LENGTH_M * cosf(h);
  }
}

// ================= Bluetooth =================

class NodeScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (!dev->isAdvertisingService(NODE_UUID)) return;
    std::string m = dev->getManufacturerData();
    if (m.size() != 2 + sizeof(NodeBeacon)) return;
    NodeBeacon b;
    memcpy(&b, m.data() + 2, sizeof(b));
    unsigned long t = millis();
    if (b.nodeType == NODE_HUB) lastHubMs = t;
    else if (b.nodeType == NODE_ANCHOR && b.nodeId >= 1 && b.nodeId <= MAX_ANCHORS)
      lastAnchorMs[b.nodeId] = t;
  }
};

void updateAdvertisement() {
  TrackerStatus s = {};
  s.magic      = TRACKER_MAGIC;
  s.trackerId  = TRACKER_ID;
  s.seq        = seq++;
  s.battery    = readBatteryPercent();
  s.flags      = (isMoving()    ? FLAG_MOVING  : 0) | (sensorFault   ? FLAG_FAULT : 0)
               | (isManDown()   ? FLAG_MANDOWN : 0) | (isHeatAlert() ? FLAG_HEAT  : 0);
  s.altDm      = (int16_t)constrain(relAlt * 10.0f, -32000.0f, 32000.0f);
  s.tempC10    = (int16_t)constrain(tempC  * 10.0f, -32000.0f, 32000.0f);
  s.pressHpa10 = (uint16_t)(pressPa / 10.0f);
  s.heading    = (uint16_t)heading;
  s.steps      = steps;
  s.pdrXdm     = (int16_t)constrain(pdrE * 10.0f, -32000.0f, 32000.0f);
  s.pdrYdm     = (int16_t)constrain(pdrN * 10.0f, -32000.0f, 32000.0f);

  uint8_t mfr[2 + sizeof(s)];
  mfr[0] = COMPANY_ID & 0xFF;
  mfr[1] = COMPANY_ID >> 8;
  memcpy(mfr + 2, &s, sizeof(s));

  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.setManufacturerData(std::string((char *)mfr, sizeof(mfr)));

  pAdvertising->stop();
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->start();
}

void setupBLE() {
  char name[12];
  snprintf(name, sizeof(name), "Tracker%02d", TRACKER_ID);
  NimBLEDevice::init(name);
  NimBLEDevice::setPower(TX_POWER_DBM);

  pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setMinInterval(32);   // 20 ms
  pAdvertising->setMaxInterval(64);   // 40 ms
  updateAdvertisement();

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new NodeScanCallbacks(), true);
  scan->setDuplicateFilter(false);
  scan->setActiveScan(false);
  scan->setInterval(400);
  scan->setWindow(100);
  scan->start(0, false);
}

// Outputs 
void updateLed(unsigned long now) {
  if (isManDown() || isHeatAlert()) {               // alarm: fast red flash
    if ((now / 250) % 2) setRGB(255, 0, 0); else setRGB(0, 0, 0);
    return;
  }
  bool blinkOff = (now % HEARTBEAT_MS) < HEARTBEAT_OFF_MS;
  if (blinkOff)          setRGB(0, 0, 0);
  else if (sensorFault)  setRGB(255, 0, 0);
  else if (linkUp())     setRGB(0, 255, 0);
  else                   setRGB(255, 160, 0);
}

void drawDisplay() {
  char link[24];
  buildLinkLine(link, sizeof(link));
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(link);
  display.drawFastHLine(0, 9, SCREEN_WIDTH, SSD1306_WHITE);
  display.setCursor(0, 12);

  if (isManDown() || isHeatAlert()) {                // alerts take over the screen
    display.setTextSize(2);
    if (isManDown())   display.println("MAN DOWN");
    if (isHeatAlert()) display.println("HEAT!");
    display.setTextSize(1);
    display.printf("%.1fC %+.1fC/min\n", tempC, tempRisePerMin);
  } else if (showMotionPage) {
    display.println("-- Position --");
    display.printf("Heading: %d %s\n", (int)heading, compassPoint(heading));
    display.printf("Steps: %u\n", steps);
    display.printf("Moved E%.1f N%.1f m\n", pdrE, pdrN);
    display.printf("State: %s\n", isMoving() ? "MOVING" : "still");
  } else {
    display.println("-- Environment --");
    display.printf("Temp: %.1fC %+.1f/min\n", tempC, tempRisePerMin);
    display.printf("Press: %.1f hPa\n", pressPa / 100.0f);
    display.printf("Alt: %.1fm Floor: %d\n", relAlt, floorNumber());
    display.println("Heat: OK");
    if (sensorFault) display.println("! SENSOR FAULT");
  }
  display.display();
}

void printSerial() {
  Serial.printf("HDG:%.0f,STEPS:%u,PDR_E:%.2f,PDR_N:%.2f,MOV:%d,MDOWN:%d,"
                "T:%.1f,RISE:%.1f,P:%.1f,ALT:%.2f,FLOOR:%d,LINK:%d,"
                "MX:%.1f,MY:%.1f,MZ:%.1f\n",
                heading, steps, pdrE, pdrN, isMoving(), isManDown(),
                tempC, tempRisePerMin, pressPa / 100.0f, relAlt, floorNumber(), linkUp(),
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
}

// ================= Setup =================

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  pixel.begin();
  pixel.setBrightness(50);
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));   // blue = booting
  pixel.show();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK);
  Wire.setTimeOut(50);

  displayOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (displayOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.printf("Booting Tracker %d\n", TRACKER_ID);
    display.display();
  } else {
    Serial.println("SSD1306 not found — continuing without display");
  }

  if (!icm.begin_I2C(ICM_ADDR, &Wire)) haltWithError("IMU not found");
  icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);
  bootMsg("IMU OK");

  if (!bmp.begin_I2C(BMP_ADDR, &Wire)) haltWithError("Barometer not found");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  bootMsg("Barometer OK");

  if (captureBaseline()) bootMsg("Altitude baseline OK");
  else { sensorFault = true; bootMsg("Baseline FAILED"); }
  processBaro(millis());            // first environment reading

  delay(800);                       // pause to read boot screen, before Bluetooth starts

  setupBLE();
  bootMsg("Bluetooth OK");
  bootMsg("Searching for Hub...");
  lastMotionMs = millis();          // 30 s grace before MAN DOWN can trigger
}

// ================= Loop =================

void loop() {
  unsigned long now = millis();

  checkResetButton();
  updateLed(now);

  if (now - lastImu >= IMU_INTERVAL_MS)       { lastImu = now;    processImu(now); }
  if (now - lastBaro >= BARO_INTERVAL_MS)     { lastBaro = now;   processBaro(now); }
  if (now - lastStatus >= STATUS_INTERVAL_MS) { lastStatus = now; updateAdvertisement(); }
  if (now - lastPrint >= PRINT_INTERVAL_MS)   { lastPrint = now;  printSerial(); }

  if (now - lastPageSwitch >= PAGE_INTERVAL) {
    showMotionPage = !showMotionPage;
    lastPageSwitch = now;
  }
  if (displayOK && now - lastDisplay >= DISPLAY_INTERVAL_MS) {
    lastDisplay = now;
    drawDisplay();
  }
}