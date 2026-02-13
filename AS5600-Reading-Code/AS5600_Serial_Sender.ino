/*
  AS5600 Magnetic Encoder Serial Sender
  Board: Arduino (Uno/Nano/etc.)
  Hardware: AS5600 Magnetic Encoder
  Purpose: Reads AS5600 angle and sends it via Serial to ESP32 on pin 12
  Sends angle in degrees as a single line: "45.2345\n"
*/

#include <Wire.h>

// === AS5600 Configuration ===
const uint8_t AS5600_ADDR = 0x36;

// === Serial Configuration ===
const unsigned long SERIAL_BAUD = 115200;
const unsigned long SEND_INTERVAL_MS = 50;  // Send angle every 50ms (20 Hz)

// === Globals ===
unsigned long lastSendTime = 0;

// === AS5600 Read Functions ===
uint8_t read8(uint8_t reg) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, (uint8_t)1);
  return Wire.read();
}

uint16_t read16(uint8_t regMSB) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(regMSB);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return ((uint16_t)msb << 8) | lsb;
}

float readAngleDegrees() {
  uint16_t raw = read16(0x0C) & 0x0FFF;  // 12-bit raw angle (0-4095)
  return (raw * 360.0f) / 4096.0f;       // Convert to degrees (0-360)
}

// === Setup ===
void setup() {
  // Initialize Serial for communication with ESP32
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for native USB)
  }

  // Initialize I2C for AS5600
  Wire.begin();
  delay(100);

  // Check AS5600 connection
  Wire.beginTransmission(AS5600_ADDR);
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("ERROR: AS5600 not found on I2C bus!");
    Serial.println("Check wiring and I2C address.");
  } else {
    Serial.println("AS5600 detected. Sending angle data...");
  }
}

// === Main Loop ===
void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastSendTime >= SEND_INTERVAL_MS) {
    // Read angle from AS5600
    float angleDeg = readAngleDegrees();
    
    // Send angle to ESP32 via Serial
    Serial.println(angleDeg, 4);  // Send with 4 decimal places
    
    lastSendTime = currentTime;
  }

  delay(5);  // Small delay to prevent I2C bus overload
}
