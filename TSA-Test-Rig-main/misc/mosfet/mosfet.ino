#define LEDC_RESOLUTION_BITS 10
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1) 
#define LEDC_FREQUENCY 5000 
#define MAX_PWM (LEDC_RESOLUTION / 5) // x% of the maximum resolution

const int mosfetPin = 33; 

int currentSpeed = 0;
bool increasing = true;
int speedStep = 10;
int delayTime = 50;

void setup() {
  Serial.begin(115200);
  pinMode(mosfetPin, OUTPUT);
  ledcAttach(mosfetPin, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcWrite(mosfetPin, 0);
}

void loop() {
  if (increasing) {
    currentSpeed += speedStep;
    if (currentSpeed >= MAX_PWM) {
      currentSpeed = MAX_PWM;
      increasing = false;
    }
  } else {
    currentSpeed -= speedStep;
    if (currentSpeed <= 0) {
      currentSpeed = 0;
      increasing = true;
      delay(500);
    }
  }
  ledcWrite(mosfetPin, currentSpeed);
  Serial.print("PWM Value: ");
  Serial.println(currentSpeed);
  delay(delayTime);
}
