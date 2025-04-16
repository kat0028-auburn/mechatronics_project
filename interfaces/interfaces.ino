#define LEFT_MOTOR_1 22
#define LEFT_MOTOR_2 23
#define LEFT_MOTOR_3 24
#define LEFT_MOTOR_4 25

#define RIGHT_MOTOR_1 26
#define RIGHT_MOTOR_2 27
#define RIGHT_MOTOR_3 28
#define RIGHT_MOTOR_4 29

uint8_t step_number = 0;

void setup() {
  pinMode(LEFT_MOTOR_1, OUTPUT);
  pinMode(LEFT_MOTOR_2, OUTPUT);
  pinMode(LEFT_MOTOR_3, OUTPUT);
  pinMode(LEFT_MOTOR_4, OUTPUT);
  pinMode(RIGHT_MOTOR_1, OUTPUT);
  pinMode(RIGHT_MOTOR_2, OUTPUT);
  pinMode(RIGHT_MOTOR_3, OUTPUT);
  pinMode(RIGHT_MOTOR_4, OUTPUT);

  Serial.begin(115200);

  clearStepper(LEFT_MOTOR_1, LEFT_MOTOR_2, LEFT_MOTOR_3, LEFT_MOTOR_4);
  clearStepper(RIGHT_MOTOR_1, RIGHT_MOTOR_2, RIGHT_MOTOR_3, RIGHT_MOTOR_4);
}

void loop() {
  turnMotor(false, 2048, LEFT_MOTOR_1, LEFT_MOTOR_2, LEFT_MOTOR_3, LEFT_MOTOR_4);
  delay(1000);
  turnMotor(false, 2048, RIGHT_MOTOR_1, RIGHT_MOTOR_2, RIGHT_MOTOR_3, RIGHT_MOTOR_4);
  delay(1000);
  turnMotor(true, 2048, LEFT_MOTOR_1, LEFT_MOTOR_2, LEFT_MOTOR_3, LEFT_MOTOR_4);
  delay(1000);
  turnMotor(true, 2048, RIGHT_MOTOR_1, RIGHT_MOTOR_2, RIGHT_MOTOR_3, RIGHT_MOTOR_4);
  delay(1000);
}

void turnMotor(const bool& dir, const int& steps, const int& pin1, const int& pin2, const int& pin3, const int& pin4) {
  for (int i = 0; i < steps; ++i)
  {
    turnOneStep(dir, pin1, pin2, pin3, pin4);
    delay(2);
  }
}

void turnOneStep(const bool& dir, const int& pin1, const int& pin2, const int& pin3, const int& pin4)
{
  if (dir) {
    switch (step_number) {
      case 0:
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
        digitalWrite(pin3, LOW);
        digitalWrite(pin4, LOW);
        break;
      case 1:
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        digitalWrite(pin3, LOW);
        digitalWrite(pin4, LOW);
        break;
      case 2:
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        digitalWrite(pin3, HIGH);
        digitalWrite(pin4, LOW);
        break;
      case 3:
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        digitalWrite(pin3, LOW);
        digitalWrite(pin4, HIGH);
        break;
    }
  }
  else {
    switch (step_number) {
      case 0:
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        digitalWrite(pin3, LOW);
        digitalWrite(pin4, HIGH);
        break;
      case 1:
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        digitalWrite(pin3, HIGH);
        digitalWrite(pin4, LOW);
        break;
      case 2:
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        digitalWrite(pin3, LOW);
        digitalWrite(pin4, LOW);
        break;
      case 3:
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
        digitalWrite(pin3, LOW);
        digitalWrite(pin4, LOW);
        break;
    }
  }
  step_number++;
  if (step_number > 3) {
    step_number = 0;
  }
}

void clearStepper(const int& pin1, const int& pin2, const int& pin3, const int& pin4) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, LOW);
  digitalWrite(pin4, LOW);
}
