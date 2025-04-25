#include <Arduino.h>
#include <HCSR04.h>
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

// Create an HCSR04 object (trigger pin, echo pin)
UltraSonicDistanceSensor sonar_1(sonar_trigger_1, sonar_echo_1);
UltraSonicDistanceSensor sonar_2(sonar_trigger_2, sonar_echo_2);
UltraSonicDistanceSensor sonar_3(sonar_trigger_3, sonar_echo_3);
double distance_1;
double distance_2;
double distance_3;
std::string range_csv;

std::string motor_csv;
int left_step_number = 0;
int right_step_number = 0;

void runSonars();
void runMotors();
void oneStep(bool dir, const int& pin1, const int& pin2, const int& pin3, const int& pin4);
void oneStepLeft(bool dir);
void oneStepRight(bool dir);
void clearSteppers();
bool checkValidMeasurement(const double& distance);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  pinMode(13, OUTPUT);
  pinMode(stepper_left_1, OUTPUT);
  pinMode(stepper_left_2, OUTPUT);
  pinMode(stepper_left_3, OUTPUT);
  pinMode(stepper_left_4, OUTPUT);
  pinMode(stepper_right_1, OUTPUT);
  pinMode(stepper_right_2, OUTPUT);
  pinMode(stepper_right_3, OUTPUT);
  pinMode(stepper_right_4, OUTPUT);

  clearSteppers();
}

void loop() {  
  runSonars(); 
  runMotors(); 
}

void runSonars()
{
  distance_1 = sonar_1.measureDistanceCm();  
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
  clearSteppers();
  if(Serial.available())
  {
    // digitalWrite(13, HIGH);
    // delay(500);
    // digitalWrite(13, LOW);
    // delay(500);

    //String message = Serial.readStringUntil('\n');
    String message = Serial.readString();
    int drive_mode;
    int drive_value;

    message.trim();
    int comma_index = message.indexOf(',');
    if (comma_index != -1)
    {
      String part1 = message.substring(0, comma_index);
      String part2 = message.substring(comma_index+1);

      drive_mode = part1.toInt();
      drive_value = part2.toInt();
    }
    else
    {
      drive_mode = 0;
    }

    if (drive_mode == 1)
    {
      for (int i = 0; i < drive_value; ++i)
      {
        oneStepLeft(true);
        oneStepRight(false);  
        delay(5);
      }
    }
    else if (drive_mode == 2)
    {
      for (int i = 0; i < drive_value; ++i)
      {
        oneStepLeft(false);
        oneStepRight(true);
        delay(5);
      }
    }
    else if (drive_mode == 3)
    {
      for (int i = 0; i < drive_value; ++i)
      {
        oneStepLeft(false);
        oneStepRight(false);
        delay(5);
      }
    }
    else if (drive_mode == 4)
    {
      for (int i = 0; i < drive_value; ++i)
      {
        oneStepLeft(true);
        oneStepRight(true);
        delay(5);
      }
    }
    else if (drive_mode == 5)
    {
      for (int i = 0; i < drive_value; ++i)
      {
        oneStepRight(false);
        delay(5);
        oneStepLeft(false);
        delay(5);
        oneStepLeft(false);
        delay(5);
      }
    }
    else if (drive_mode == 6)
    {
      for (int i = 0; i < drive_value; ++i)
      {
        oneStepLeft(true);
        delay(5);
        oneStepLeft(true);
        delay(5);
        oneStepRight(true);
        delay(5);
      }
    }
  }
}

void oneStepLeft(bool dir){
  const int pin1 = stepper_left_1;
  const int pin2 = stepper_left_2;
  const int pin3 = stepper_left_3;
  const int pin4 = stepper_left_4;
  if (dir){
    switch(left_step_number){
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
    switch(left_step_number){
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
  left_step_number++;
  if (left_step_number > 3){
    left_step_number = 0;
  }
}

void oneStepRight(bool dir){
  const int pin1 = stepper_right_1;
  const int pin2 = stepper_right_2;
  const int pin3 = stepper_right_3;
  const int pin4 = stepper_right_4;
  if (dir){
    switch(right_step_number){
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
    switch(right_step_number){
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
  right_step_number++;
  if (right_step_number > 3){
    right_step_number = 0;
  }
}

void clearSteppers()
{
  digitalWrite(stepper_left_1, LOW);
  digitalWrite(stepper_left_2, LOW);
  digitalWrite(stepper_left_3, LOW);
  digitalWrite(stepper_left_4, LOW);
  digitalWrite(stepper_right_1, LOW);
  digitalWrite(stepper_right_2, LOW);
  digitalWrite(stepper_right_3, LOW);
  digitalWrite(stepper_right_4, LOW);

  left_step_number = 0;
  right_step_number = 0;
}