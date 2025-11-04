#include <Arduino.h>
#include <Servo.h>

#define TRIG_PIN 6
#define ECHO_PIN 7

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Initialize Serial at 115200 baud for communication with Jetson
  Serial.begin(115200);
  
  delay(2000);
}

void loop() { 
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  float duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (duration / 2) / 74.0;

  // Send data in a parseable format: "DIST:<value>\n"
  Serial.print("DIST:");
  Serial.println(distance, 2); // 2 decimal places
  
  delay(100); // 10Hz update rate
}