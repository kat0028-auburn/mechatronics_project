#include <Arduino.h>
#include <HCSR04.h>

// Create an HCSR04 object (trigger pin, echo pin)
UltraSonicDistanceSensor sensor(9, 10);

void setup() {
  Serial.begin(115200);
}

void loop() {
  float distance = sensor.measureDistanceCm();  // Returns distance in centimeters

  if (!isnan(distance)) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  } else {
    Serial.println("Measurement failed");
  }

  delay(10);
}
