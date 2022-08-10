#include "Arduino.h"

int dacPin = DAC0;
int analogPin = A0;
int val = 0;

void setup() {
  Serial.begin(115200);
  // Turn on Status LED to shows board is on.
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, HIGH);
  Serial.println("Coral Micro Arduino DAC Write!");
}

void loop() {
  val = analogRead(analogPin);
  analogWrite(dacPin, val / 4);
}
