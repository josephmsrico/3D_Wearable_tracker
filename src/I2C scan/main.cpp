#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(17, 18);        // SDA = chip pin 17, SCL = chip pin 18
  Wire.setClock(50000);
  Wire.setTimeOut(50);
}

void loop() {
  Serial.println("\nScanning I2C bus...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found device at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) Serial.println("  No devices found!");
  delay(3000);
}