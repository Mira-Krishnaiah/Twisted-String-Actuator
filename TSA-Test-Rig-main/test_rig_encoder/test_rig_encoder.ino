/*
  Hardware Code for Twisted String Actuator Test Rig
  Board: ESP32-S3 DEV KIT (WROOM-1)
  Sensors required: IR sensors (2x), INA3221 (current sensor) or equivalent
  Motor: Coreless motor
  Driver: MOSFET H Bridge (IRLB8721 or equivalent)
  Power: Check motor specs for voltage and current ratings
*/

#include <Wire.h>
#include <Adafruit_INA3221.h>
#include <HardwareSerial.h>

#define LEDC_RESOLUTION_BITS 10
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQUENCY 500
#define MAX_PWM (LEDC_RESOLUTION)

#define MANUAL_CONTROL 0

const int pinA = 6;
const int pinB = 7;

const int SDA1 = 8;
const int SCL1 = 9;
const int IR_SENSOR_1 = 10;
const int IR_SENSOR_2 = 11;
const int ARDUINO_SERIAL_RX = 12;  // ESP32 RX pin connected to Arduino TX

Adafruit_INA3221 ina3221;

// Serial communication with Arduino for AS5600 angle data
HardwareSerial ArduinoSerial(2);  // Use Serial2 (UART2) on ESP32

int delayTime = 10;
unsigned long cycleCount = 0;
#define LOG_INTERVAL 10 
unsigned long lastLogTime = 0;

// AS5600 angle data from Arduino
float angleDeg = 0.0f;
bool angleDataValid = false;
unsigned long lastAngleUpdateTime = 0;
#define ANGLE_TIMEOUT_MS 1000  // If no angle data for 1 second, mark as invalid

enum AutoState { STATE_EXTENDING, STATE_RETRACTING };
AutoState autoState = STATE_EXTENDING;

// === Fault Detection Settings ===
#define SENSOR_STALL_TIMEOUT_MS    2000
#define PIN_RESET                  13  // Changed from 12 to avoid conflict with Serial RX

unsigned long sensorLastChangedTime = 0;
int lastIR1 = HIGH;
int lastIR2 = HIGH;

bool wireBreakDetected = false;

// Read angle data from Arduino via Serial
void readAngleFromArduino() {
  static String angleBuffer = "";
  
  while (ArduinoSerial.available()) {
    char c = ArduinoSerial.read();
    
    if (c == '\n' || c == '\r') {
      if (angleBuffer.length() > 0) {
        // Try to parse angle value
        float parsedAngle = angleBuffer.toFloat();
        if (parsedAngle >= 0.0f && parsedAngle <= 360.0f) {
          angleDeg = parsedAngle;
          angleDataValid = true;
          lastAngleUpdateTime = millis();
        }
        angleBuffer = "";
      }
    } else if (isDigit(c) || c == '.' || c == '-') {
      angleBuffer += c;
    } else {
      // Reset buffer on unexpected character
      angleBuffer = "";
    }
  }
  
  // Check if angle data is stale
  if (millis() - lastAngleUpdateTime > ANGLE_TIMEOUT_MS) {
    angleDataValid = false;
  }
}

void setup() {  
  Serial.begin(115200);

  Wire.begin(SDA1, SCL1);
  if (!ina3221.begin()) {
    Serial.println("Couldn't find INA3221 chip");
    while (1);
  }

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  ledcAttach(pinA, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcAttach(pinB, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcWrite(pinA, 0);
  ledcWrite(pinB, 0);

  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_2, INPUT);
  pinMode(PIN_RESET, INPUT_PULLUP);  // jumper to GND for recovery

  // Initialize Serial communication with Arduino (AS5600)
  ArduinoSerial.begin(115200, SERIAL_8N1, ARDUINO_SERIAL_RX, -1);  // RX only, no TX needed
  delay(100);  // Allow serial to initialize

  // Initialize sensor tracking
  lastIR1 = digitalRead(IR_SENSOR_1);
  lastIR2 = digitalRead(IR_SENSOR_2);
  sensorLastChangedTime = millis();

  Serial.println("Setup complete.");
  Serial.println("Waiting for Arduino AS5600 data...");
  Serial.println("-----------");

}

void loop() {
  // Read angle data from Arduino
  readAngleFromArduino();
  
  Serial.println("-----------");
  float currentValue = ina3221.getCurrentAmps(2) * 1000; // mA
  float voltageValue = ina3221.getBusVoltage(2) * 1000;  // mV

  bool motorActive = (autoState == STATE_EXTENDING && ledcRead(pinA) > 0) ||
                     (autoState == STATE_RETRACTING && ledcRead(pinB) > 0);

  // === Fault Handling ===
  if (wireBreakDetected) {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, 0);

    if (digitalRead(PIN_RESET) == LOW) {
      Serial.println("Recovery signal received. Resuming operation.");
      wireBreakDetected = false;
      sensorLastChangedTime = millis();  // reset timer
    }

    delay(delayTime);
    return;
  }

  // === Motor Control ===
  if (autoState == STATE_EXTENDING) {
    if (digitalRead(IR_SENSOR_2) == LOW) {
      ledcWrite(pinA, 0);
      delay(1);
      ledcWrite(pinB, MAX_PWM * 0.05);  // light back-drive
      autoState = STATE_RETRACTING;
    }
  } else if (autoState == STATE_RETRACTING) {
    if (digitalRead(IR_SENSOR_1) == LOW) {
      ledcWrite(pinB, 0);
      delay(850);
      ledcWrite(pinA, MAX_PWM);
      cycleCount++;
      autoState = STATE_EXTENDING;
      Serial.print(cycleCount);
      Serial.print(",");
      Serial.print(currentValue);
      Serial.print(",");
      Serial.print(voltageValue);
      Serial.print(",");
      Serial.println(angleDataValid ? angleDeg : 0.0f);
      lastLogTime = millis();
    }
  } else {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, 0);
  }

  // === Sensor Change Timeout Detection ===
  int currentIR1 = digitalRead(IR_SENSOR_1);
  int currentIR2 = digitalRead(IR_SENSOR_2);

  if (currentIR1 != lastIR1 || currentIR2 != lastIR2) {
    sensorLastChangedTime = millis();
    lastIR1 = currentIR1;
    lastIR2 = currentIR2;
  }

  if (motorActive && millis() - sensorLastChangedTime > SENSOR_STALL_TIMEOUT_MS) {
    wireBreakDetected = true;
  }

  // === Logging ===
  if (millis() - lastLogTime >= LOG_INTERVAL) {
    Serial.print(cycleCount);
    Serial.print(",");
    Serial.print(currentValue);
    Serial.print(",");
    Serial.print(voltageValue);
    Serial.print(",");
    Serial.println(angleDataValid ? angleDeg : 0.0f);
    lastLogTime = millis();
  }

  

  delay(delayTime);
}
