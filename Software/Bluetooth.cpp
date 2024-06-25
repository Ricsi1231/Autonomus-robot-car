#include "SerialUART.h"
#include "Bluetooth.h"
#include <string.h>
#include <Arduino.h>

Bluetooth::Bluetooth() {

}

bool Bluetooth::Init() {
  Serial1.begin(9600);

  while(!Serial1);

  return false;
}

char Bluetooth::ReadMessage() {
  char Data;

  if(Serial1.available() > 0) {
    Data = Serial1.read();
  }

  return Data;
}

String Bluetooth::ReadString() {
  String message = "";

  if(Serial.available() > 0) {
    message = Serial1.readString();
  }

  return message;
}
