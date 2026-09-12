#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP3XX.h>
#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>

#define SDA_PIN 17
#define SCL_PIN 18
#define I2C_CLOCK 50000UL

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define ICM_ADDR 0x69
#define BMP_ADDR 0x77

#define TRACKER_SERVICE_UUID "5f524546-4c45-5f42-4c45-5f5452434b52"

#define RGB_PIN 21
#define NUM_PIXELS 1

Adafruit_ICM20948 icm;
Adafruit_BMP3XX bmp;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_NeoPixel pixel(NUM_PIXELS, RGB_PIN, NEO_RGB + NEO_KHZ800);

sensors_event_t accel, gyro, temp, mag;

bool showMotionPage = true;
unsigned long lastPageSwitch = 0;
const unsigned long PAGE_INTERVAL = 5000; // switch page every 5 seconds

unsigned long lastAdvFlash = 0;
const unsigned long ADV_FLASH_INTERVAL = 500; // flash blue every 500ms to show active transmission

NimBLEAdvertising *pAdvertising;

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

void setupBLE() {
  NimBLEDevice::init("Tracker01");
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);

  pAdvertising = NimBLEDevice::getAdvertising();

  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.setCompleteServices(NimBLEUUID(TRACKER_SERVICE_UUID));
  pAdvertising->setAdvertisementData(advData);

  pAdvertising->start();
  Serial.println("BLE advertising started.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(500);

  pixel.begin();
  pixel.setBrightness(50);
  setRGB(0, 0, 255); // Blue — booting

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK);
  Wire.setTimeOut(50);

  if (!icm.begin_I2C(ICM_ADDR, &Wire)) {
    while (1) {
      setRGB(255, 0, 0);
      Serial.println("ICM-20948 not found!");
      delay(1000);
    }
  }
  Serial.println("ICM-20948 initialized.");

  if (!bmp.begin_I2C(BMP_ADDR, &Wire)) {
    while (1) {
      setRGB(255, 0, 0);
      Serial.println("BMP390 not found!");
      delay(1000);
    }
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("BMP390 initialized.");

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (1) {
      setRGB(255, 0, 0);
      Serial.println("SSD1306 not found!");
      delay(1000);
    }
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("IMU + BMP + OLED ready");
  display.display();
  delay(1000);

  setupBLE();

  setRGB(0, 255, 0); // Green — everything up and advertising
}

void loop() {
  // --- Visual "sending signal" heartbeat ---
  if (millis() - lastAdvFlash >= ADV_FLASH_INTERVAL) {
    lastAdvFlash = millis();
    setRGB(0, 0, 255); // brief blue flash — actively broadcasting
    delay(30);         // short flash duration
    setRGB(0, 255, 0); // return to green (normal operating state)
  }

  // --- Automatic page switch every 5 seconds ---
  if (millis() - lastPageSwitch >= PAGE_INTERVAL) {
    showMotionPage = !showMotionPage;
    lastPageSwitch = millis();
  }

  // --- Sensor reading ---
  icm.getEvent(&accel, &gyro, &temp, &mag);
  bool bmpOK = bmp.performReading();

  Serial.print("AX:"); Serial.print(accel.acceleration.x, 2);
  Serial.print(",AY:"); Serial.print(accel.acceleration.y, 2);
  Serial.print(",AZ:"); Serial.print(accel.acceleration.z, 2);
  Serial.print(",GX:"); Serial.print(gyro.gyro.x, 2);
  Serial.print(",GY:"); Serial.print(gyro.gyro.y, 2);
  Serial.print(",GZ:"); Serial.print(gyro.gyro.z, 2);

  if (bmpOK) {
    Serial.print(",TEMP:"); Serial.print(bmp.temperature, 2);
    Serial.print(",PRESS:"); Serial.print(bmp.pressure / 100.0, 2);
    Serial.print(",ALT:"); Serial.println(bmp.readAltitude(1013.25), 2);
  } else {
    Serial.println(",BMP:READ_FAIL");
  }

  // --- OLED display ---
  display.clearDisplay();
  display.setCursor(0, 0);

  if (showMotionPage) {
    display.println("Motion");
    display.printf("AX:%.2f AY:%.2f\n", accel.acceleration.x, accel.acceleration.y);
    display.printf("AZ:%.2f\n", accel.acceleration.z);
    display.printf("GX:%.2f GY:%.2f\n", gyro.gyro.x, gyro.gyro.y);
    display.printf("GZ:%.2f\n", gyro.gyro.z);
  } else {
    display.println("Environment");
    if (bmpOK) {
      display.printf("Temp:%.2fC\n", bmp.temperature);
      display.printf("Press:%.2fhPa\n", bmp.pressure / 100.0);
      display.printf("Alt:%.2fm\n", bmp.readAltitude(1013.25));
    } else {
      display.println("Read failed");
    }
  }

  display.display();
  delay(50);
}