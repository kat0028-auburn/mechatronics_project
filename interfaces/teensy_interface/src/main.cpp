#include <Arduino.h>
#include <HCSR04.h>
#include <Stepper.h>
#include <string>

const int sonar_trigger_1 = 9;
const int sonar_echo_1 = 10;
const int sonar_trigger_2 = 24;
const int sonar_echo_2 = 25;
const int sonar_trigger_3 = 31;
const int sonar_echo_3 = 32;

const int stepper_left_1 = 35;
const int stepper_left_2 = 36;
const int stepper_left_3 = 37;
const int stepper_left_4 = 38;
const int stepper_right_1 = 15;
const int stepper_right_2 = 16;
const int stepper_right_3 = 17;
const int stepper_right_4 = 18;
const int steps = 2048;

// Create an HCSR04 object (trigger pin, echo pin)
UltraSonicDistanceSensor sonar_1(sonar_trigger_1, sonar_echo_1);
UltraSonicDistanceSensor sonar_2(sonar_trigger_2, sonar_echo_2);
UltraSonicDistanceSensor sonar_3(sonar_trigger_3, sonar_echo_3);
double distance_1;
double distance_2;
double distance_3;
std::string range_csv;

Stepper stepper_left(steps, stepper_left_1, stepper_left_2, stepper_left_3, stepper_left_4);
Stepper stepper_right(steps, stepper_right_1, stepper_right_2, stepper_right_3, stepper_right_4);
std::string motor_csv;
int left_steps;
int right_steps;
int step_number = 0;

void runSonars();
void runMotors();
void oneStep(bool dir, const int& pin1, const int& pin2, const int& pin3, const int& pin4);
bool checkValidMeasurement(const double& distance);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  pinMode(13, OUTPUT);
}

void loop() {
  //runSonars(); 
  runMotors(); 
}

void runSonars()
{
  distance_1 = sonar_1.measureDistanceCm();  // Returns distance in centimeters
  distance_2 = sonar_2.measureDistanceCm();
  distance_3 = sonar_3.measureDistanceCm();

  if (checkValidMeasurement(distance_1) && checkValidMeasurement(distance_2) && checkValidMeasurement(distance_3)){// && checkValidMeasurement(distance_2) && checkValidMeasurement(distance_3)) {
    range_csv = std::to_string((int)distance_1) + "," + std::to_string((int)distance_2) + "," + std::to_string((int)distance_3);
    Serial.println(range_csv.c_str());
  }
}

bool checkValidMeasurement(const double& distance)
{
  return (!isnan(distance) && distance>0);
}

void runMotors()
{
  if(Serial.available())
  {
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);

    String message = Serial.readString();
    char buf[message.length() + 1];
    message.toCharArray(buf, sizeof(buf));

    char *token = strtok(buf, ",");
    int index = 0;
    int values[2];

    while (token != nullptr && index < 2)
    {
      values[index++] = atoi(token);
      token=strtok(nullptr, ",");
    }

    left_steps = values[0];
    right_steps = values[1];

    int left_counter = 0;
    int right_counter = 0;
    bool left_dir = (left_steps > 0);
    bool right_dir = !(right_steps > 0);
    bool left_turning = true;
    bool right_turning = true;

    Serial.println(left_steps);
    
    for (int i = 0; i < abs(left_steps); i++)
    {
      oneStep(left_dir, stepper_left_1, stepper_left_2, stepper_left_3, stepper_left_4);
      delay(2);
    }
    for (int i = 0; i < abs(right_steps); i++)
    {
      oneStep(right_dir, stepper_right_1, stepper_right_2, stepper_right_3, stepper_right_4);
      delay(2);
    }
  }
}

void oneStep(bool dir, const int& pin1, const int& pin2, const int& pin3, const int& pin4){
  if (dir){
    switch(step_number){
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
  else{
    switch(step_number){
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
  if (step_number > 3){
    step_number = 0;
  }
}