#include "Motor.h"
#include <Arduino.h>

Motor::Motor(uint8_t Motor1Pin1, uint8_t Motor1Pin2, uint8_t Motor1PWM, uint8_t Motor2Pin1, uint8_t Motor2Pin2, uint8_t Motor2PWM) {
  this->motor1_Pin1 = Motor1Pin1;
  this->motor1_Pin2 = Motor1Pin2;
  this->motor1PWM = Motor1PWM;

  this->motor2_Pin1 = Motor2Pin1;
  this->motor2_Pin2 = Motor2Pin2;
  this->motor2PWM = Motor2PWM;
}

bool Motor::Init() {
  if(motor1_Pin1 != 0 && motor1_Pin2 != 0 && motor1PWM != 0 && motor2_Pin1 != 0 && motor2_Pin2 != 0 && motor2PWM != 0) {
    return false;
  }
  
  pinMode(motor1_Pin1, OUTPUT);
  pinMode(motor1_Pin2, OUTPUT);
  pinMode(motor1PWM, OUTPUT);

  pinMode(motor2_Pin1, OUTPUT);
  pinMode(motor2_Pin2, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

  return true;
}

void Motor::Forward(int speed) {
  for(int i = 0; i < speed; i++) {
    digitalWrite(motor1_Pin1, LOW);
    digitalWrite(motor1_Pin2, HIGH);
    digitalWrite(motor2_Pin1, HIGH);
    digitalWrite(motor2_Pin2, LOW);

    analogWrite(motor1PWM, speed);
    analogWrite(motor2PWM, speed);
  }
}

void Motor::TurnLeft(int speed) {
  for(int i = 0; i < speed; i++) {
    digitalWrite(motor1_Pin1, HIGH);
    digitalWrite(motor1_Pin2, LOW);
    digitalWrite(motor2_Pin1, HIGH);
    digitalWrite(motor2_Pin2, LOW);

    analogWrite(motor1PWM, speed);
    analogWrite(motor2PWM, speed);
  }
}

void Motor::TurnRight(int speed) {
  for(int i = 0; i < speed; i++) {
    digitalWrite(motor1_Pin1, LOW);
    digitalWrite(motor1_Pin2, HIGH);
    digitalWrite(motor2_Pin1, LOW);
    digitalWrite(motor2_Pin2, HIGH);

    analogWrite(motor1PWM, speed);
    analogWrite(motor2PWM, speed);
  }
}

void Motor::Backward(int speed) {
  for(int i = 0; i < speed; i++) {
    digitalWrite(motor1_Pin1, HIGH);
    digitalWrite(motor1_Pin2, LOW);
    digitalWrite(motor2_Pin1, LOW);
    digitalWrite(motor2_Pin2, HIGH);

    analogWrite(motor1PWM, speed);
    analogWrite(motor2PWM, speed);
  }
}

void Motor::Stop() {
    digitalWrite(motor1_Pin1, LOW);
    digitalWrite(motor1_Pin2, LOW);
    digitalWrite(motor2_Pin1, LOW);
    digitalWrite(motor2_Pin2, LOW);
}