#include <sys/_stdint.h>
#include "HC_SR04.h"
#include <Arduino.h>

HC_SR04::HC_SR04(uint8_t TriggerPin, uint8_t EchoPin) {
  this->triggerPin = TriggerPin;
  this->echoPin = EchoPin;
}

bool HC_SR04::Init() {
  if(triggerPin != 0 && echoPin != 0) {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, OUTPUT);
  }
  else {
    return false;
  }

  return true;
}

int HC_SR04::Read() {
  uint32_t duration = 0;
  uint32_t distance = 0;

  digitalWrite(triggerPin, LOW);
  delay(2);
  digitalWrite(triggerPin, HIGH);
  delay(2);
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.034) / 2;

  return distance;
}
