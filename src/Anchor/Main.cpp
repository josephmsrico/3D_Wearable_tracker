#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>

#define TRACKER_SERVICE_UUID "5f524546-4c45-5f42-4c45-5f5452434b52"
#define RGB_PIN 21
#define NUM_PIXELS 1
#define CONNECTION_TIMEOUT_MS 4000
#define STATUS_CHECK_INTERVAL_MS 3000

NimBLEScan *pScan;
Adafruit_NeoPixel pixel(NUM_PIXELS, RGB_PIN, NEO_RGB + NEO_KHZ800);

volatile unsigned long lastSeenMs = 0;
volatile bool everSeen = false;
unsigned long lastStatusCheck = 0;

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override {
    if (advertisedDevice->isAdvertisingService(NimBLEUUID(TRACKER_SERVICE_UUID))) {
      lastSeenMs = millis();
      everSeen = true;
    }
  }
};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(500);

  pixel.begin();
  pixel.setBrightness(50);
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));
  pixel.show();

  Serial.println("Booting Anchor...");

  NimBLEDevice::init("Anchor01");
  pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new ScanCallbacks());
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);
  pScan->start(0, false);

  Serial.println("Scanning continuously...");
  delay(500);
}

void loop() {
  if (millis() - lastStatusCheck >= STATUS_CHECK_INTERVAL_MS) {
    lastStatusCheck = millis();

    bool connected = everSeen && (millis() - lastSeenMs < CONNECTION_TIMEOUT_MS);

    if (connected) {
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
      Serial.print("[");
      Serial.print(millis());
      Serial.println("] Status check: CONNECTED");
    } else {
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));
      Serial.print("[");
      Serial.print(millis());
      Serial.println("] Status check: NOT CONNECTED");
    }
    pixel.show();
  }

  delay(50);
}