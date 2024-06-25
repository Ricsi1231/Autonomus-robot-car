#include "Motor.h"
#include "Pins.h"
#include <Servo.h>
#include "Bluetooth.h"
#include "Button.h"
#include "HC_SR04.h"
#include "L3G4200D.h"

uint32_t currentTime = 0;
uint32_t previousTime = 0;

uint8_t counter = 0;

double x_axis = 0;
double y_axis = 0;
double z_axis = 0;

Bluetooth bluetooth;

Servo HC_SR04_SERVO;
Motor motors(MOTOR1_PIN_1, MOTOR1_PIN_2, MOTOR1_EN, MOTOR2_PIN_1, MOTOR2_PIN_2, MOTOR2_EN);

HC_SR04 Sensor(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);

Button Mode_Select_Button(MODE_SELECT_BUTTON, 0);

L3G4200D Gyro;

void Bluetooth_Mode();  
void Obstacle_avoiding();
void Voice_control();
void Line_follower();

void setup() {
  Serial.begin(115200);

  pinMode(INFRA_SENSOR_RIGHT, INPUT);
  pinMode(INFRA_SENSOR_LEFT, INPUT);

  if(!motors.Init()) {
    Serial.println("Motor ERROR");
  }

  if(!bluetooth.Init()) {
    Serial.println("Bluetooth ERROR");
  }

  if(!Mode_Select_Button.Init()) {
    Serial.println("Button ERROR");
  }

  if(!Sensor.Init()) {
    Serial.println("HC_SR04 ERROR");
  }

  HC_SR04_SERVO.attach(SERVO1_PIN);
}

void loop() {
  if(Mode_Select_Button.Read(500)) {
    counter ++;
    delay(200);
  }

  if(Mode_Select_Button.Read(1500)) {
    switch(counter) {
      case 1:
        Obstacle_avoiding();
      break;

      case 2:
        Bluetooth_Mode();
      break;

      case 3:
        Voice_control();
      break;

      case 4:
        Line_follower();
      break;
    }
  }
}

void Bluetooth_Mode() {
  char bluetoothCommand;

  bluetoothCommand = bluetooth.ReadMessage();

  if(bluetoothCommand == 'F') {
    motors.Forward(190);
  }

  else if(bluetoothCommand == 'B') {
    motors.Backward(190);
  }

  else if(bluetoothCommand == 'R') {
    motors.TurnRight(190);
  }

  else if(bluetoothCommand == 'L'){
    motors.TurnLeft(190);
  }

  else {
    motors.Stop();
  }

  if(Mode_Select_Button.Read(500)) {
    counter = 0;
    return;
  }
}

void Obstacle_avoiding() {
  uint32_t distance = 0;
  uint32_t distanceLeft = 0;
  uint32_t distanceRight = 0;

  distance = Sensor.Read();

  if(distance < 15) {
    currentTime = millis();

    motors.Backward(190);

    if(currentTime - previousTime >= 1000) {
      motors.Stop();

      previousTime = currentTime;
    }

    if(currentTime - previousTime > 2500) {
      HC_SR04_SERVO.write(90);
      distanceLeft = Sensor.Read();

      previousTime = currentTime;
    }

    if(currentTime - previousTime > 3500) {
      HC_SR04_SERVO.write(170);
      distanceRight = Sensor.Read();

      previousTime = currentTime;
    }

    if(currentTime - previousTime >= 4000) {
      HC_SR04_SERVO.write(80);
    }

    if(distanceRight > distanceLeft) {
        currentTime = millis();

        motors.TurnRight(190);

        if(currentTime - previousTime >= 1000) {
          motors.Stop();

          previousTime = currentTime;
        }
    }
    else {
        currentTime = millis();

        motors.TurnLeft(190);

        if(currentTime - previousTime >= 1000) {
          motors.Stop();

          previousTime = currentTime;
        }
    }
  }
  else {
    motors.Forward(190);
  }

  if(Mode_Select_Button.Read(500)) {
    counter = 0;
    currentTime = 0;
    previousTime = 0;
    return;
  }
}

void Voice_control() {
  String Data = "";

  Data = bluetooth.ReadString();

  if(Data == "*move forward#") {
    currentTime = millis();
    
    motors.Forward(190);
    
    if(currentTime - previousTime > 1000) {
      motors.Stop();

      previousTime = currentTime;
    }

    Data = "";
  }

  else if(Data == "*move backward#") {
    motors.Backward(190);

    if(currentTime - previousTime > 1000) {
      motors.Stop();

      previousTime = currentTime;
    }

    Data = "";
  }

  else if(Data == "*turn left#") {
      Gyro.Read(x_axis, y_axis, z_axis);

      if(x_axis != 90) {
        Gyro.Read(x_axis, y_axis, z_axis);

        motors.TurnLeft(190);
      }

    if(currentTime - previousTime > 1000) {
      motors.Stop();

      previousTime = currentTime;
    }

    Data = "";
  }

  else if(Data == "*turn right#") {
      if(x_axis != 90) {
        Gyro.Read(x_axis, y_axis, z_axis);

        motors.TurnRight(190);
      }

    if(currentTime - previousTime > 1000) {
      motors.Stop();

      previousTime = currentTime;
    }

    Data = "";
  }

  else if(Data == "*stop#") {
    motors.Stop();

    Data = "";
  }

  if(Mode_Select_Button.Read(500)) {
    counter = 0;
    currentTime = 0;
    previousTime = 0;
    return;
  }
} 

void Line_follower() {
  if(analogRead(INFRA_SENSOR_RIGHT) <= 35 && analogRead(INFRA_SENSOR_LEFT) <= 35)
    motors.Forward(100);

  else if(analogRead(INFRA_SENSOR_RIGHT) <= 35 && !analogRead(INFRA_SENSOR_LEFT) <= 35)
    motors.TurnRight(255);

  else if(!analogRead(INFRA_SENSOR_RIGHT) <= 35 && analogRead(INFRA_SENSOR_LEFT) <= 35)
    motors.TurnRight(255);

  else if(!analogRead(INFRA_SENSOR_RIGHT) <= 35 && !analogRead(INFRA_SENSOR_LEFT) <= 35)
    motors.Stop();

  if(Mode_Select_Button.Read(500)) {
    counter = 0;
    return;
  }
}
