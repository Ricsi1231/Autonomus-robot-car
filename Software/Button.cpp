#include "Button.h"
#include <Arduino.h>

Button::Button(uint8_t Pin, uint8_t PullResistor) {
  this->pin = Pin;
  this->pullResistor = PullResistor;
}

bool Button::Init() {
  if(pin != 0) {
    if(pullResistor == 0) {
      pinMode(pin, INPUT);
    }
    else if(pullResistor == 1) {
      pinMode(pin, INPUT_PULLUP);
    }
    else {
      pinMode(pin, INPUT_PULLDOWN);
    }
  }
  else {
    return false;
  }

  return true;
}

int Button::Read(int Time) {
  uint32_t currentTime = 0;
  uint32_t previousTime = 0;
  
  uint8_t state = 0;
  uint8_t buttonState = 0;

  if(pullResistor == 0 || pullResistor == 2) {
    currentTime = millis();

    if(currentTime - previousTime > Time && digitalRead(pin)) {
      buttonState = 1;
      previousTime = currentTime;
      delay(10);
    }

    state = buttonState;
    buttonState = 0;
  }

  if(pullResistor == 1) {
    currentTime = millis();

    if(currentTime - previousTime > Time && !digitalRead(pin)) {
      buttonState = 1;
      previousTime = currentTime;
      delay(10);
    }

    state = buttonState;
    buttonState = 0;
  }

  return state;
}