/*
  Version of test rig with multiple sensors:
  - APDS9960 proximity sensor
  - VL53L0X time-of-flight distance sensor
  - Two IR sensors for end-stop detection

  NOT NEEDED ANYMORE
*/

#include <Adafruit_APDS9960.h>
#include <Adafruit_VL53L0X.h>
#define LEDC_RESOLUTION_BITS 10
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1) // 10-bit resolution (1023)
#define LEDC_FREQUENCY 5000 
#define MAX_PWM (LEDC_RESOLUTION / 4) // x% of the maximum resolution

#define MANUAL_CONTROL 0 // 1 = manual control, 0 = auto mode using sensor feedback
const int potPin = 37; 
const int mosfetPin = 33;  // PWM pin connected to MOSFET gate
const int SDA1 = 22; // I2C SDA pin
const int SCL1 = 20; // I2C SCL pin
const int IR_SENSOR_1 = 14; // IR sensor 1 pin
const int IR_SENSOR_2 = 32; // IR sensor 2 pin

Adafruit_APDS9960 apds = Adafruit_APDS9960();
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

int currentSpeed = 0;
bool calibrationMode = true;
int potMin = 60;   // Default low endpoint
int potMax = 2048; // Default high endpoint
int delayTime = 10; // ms delay
unsigned long cycleCount = 0; // Added to track full cycles

// Auto-mode state machine
enum AutoState { STATE_EXTENDING, STATE_RETRACTING };
AutoState autoState = STATE_EXTENDING;

void setup() {  
  Serial.begin(115200);
  Serial.println("\n\n");  // Clear serial monitor
  Serial.println("Setting up test rig...");
  pinMode(mosfetPin, OUTPUT);
  ledcAttach(mosfetPin, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcWrite(mosfetPin, 0); 
  Serial.println("Motor control ready.");
  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_2, INPUT);
  if (digitalRead(IR_SENSOR_1) == -1 || digitalRead(IR_SENSOR_2) == -1) {
    Serial.println("Failed to initialize IR sensors.");
  } else {
    Serial.print("IR sensors initialized on pins ");
    Serial.print(IR_SENSOR_1); Serial.print(" and "); Serial.println(IR_SENSOR_2);
  }
  Wire.begin(SDA1, SCL1);

  // Initialize APDS9960 proximity sensor
  if (apds.begin()) {
    apds.enableProximity(true);
    Serial.println("APDS9960 initialized successfully on I2C");
  } else {
    Serial.println("APDS9960 init failed");
  }

  // Initialize VL53L0X TOF sensor
  if (lox.begin()) {
    // Optional: lox.setMeasurementTimingBudgetMicroSeconds(50000);
    Serial.println("VL53L0X initialized successfully on I2C");
  } else {
    Serial.println("VL53L0X init failed");
  }
  pinMode(potPin, INPUT);
  Serial.println("Setup complete.");
}

void loop() {
#if MANUAL_CONTROL
  // Added calibration branch for manual control
  if (calibrationMode) {
    int potValue = analogRead(potPin);
    Serial.print("Calibration - Pot Value: ");
    Serial.println(potValue);
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      if (cmd == 'L' || cmd == 'l') {
        potMin = potValue;
        Serial.print("Low endpoint set to: ");
        Serial.println(potMin);
        Serial.println("Now set slider to HIGH endpoint and type 'H' then press ENTER.");
      } else if (cmd == 'H' || cmd == 'h') {
        potMax = potValue;
        Serial.print("High endpoint set to: ");
        Serial.println(potMax);
        calibrationMode = false;
        Serial.println("Calibration complete! Normal operation started.");
      }
    }
    delay(500);
    return;
  } else {
    int potVal = analogRead(potPin);
    int pwmVal = map(potVal, potMin, potMax, 0, MAX_PWM);
    pwmVal = constrain(pwmVal, 0, MAX_PWM);
    ledcWrite(mosfetPin, pwmVal);
  }
#else
  // Auto mode: passively extend until IR2, actively retract until IR1
  switch(autoState) {
    case STATE_EXTENDING:
      if (digitalRead(IR_SENSOR_2) == LOW) {
        ledcWrite(mosfetPin, 0); // motor off, let weight back‑drive
      } else {
        ledcWrite(mosfetPin, 0);
        autoState = STATE_RETRACTING; // prepare to retract
      }
      break;

    case STATE_RETRACTING:
      if (digitalRead(IR_SENSOR_1) == LOW) {
        ledcWrite(mosfetPin, MAX_PWM); // retract until full retraction
      } else {
        ledcWrite(mosfetPin, 0);
        cycleCount++;
        autoState = STATE_EXTENDING; // prepare to extend
      }
      break;
  }
#endif

  // display sensor readings
  uint8_t prox = apds.readProximity();
  uint16_t distance = 0;
  // Single-shot distance measurement from VL53L0X
  distance = lox.readRangeSingleMillimeters();

  int irVal1 = digitalRead(IR_SENSOR_1);
  int irVal2 = digitalRead(IR_SENSOR_2);
  Serial.print(cycleCount);
  Serial.print(",");
  Serial.print(currentSpeed);
  Serial.print(",");
  Serial.print(prox);
  Serial.print(",");
  Serial.print(distance);
  Serial.print(",");
  Serial.print(irVal1);
  Serial.print(",");
  Serial.println(irVal2);
  delay(delayTime);
}
