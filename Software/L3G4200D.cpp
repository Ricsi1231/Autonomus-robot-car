#include <sys/_stdint.h>
#include "L3G4200D.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

L3G4200D::L3G4200D() {

}

bool L3G4200D::Init() {
  Wire1.begin();
  Wire1.beginTransmission(I2C_ADDRESS);

  Wire1.write(CTRL_REG1);
  Wire1.write(0x0F);
  Wire1.endTransmission();

  Wire1.beginTransmission(I2C_ADDRESS);
  Wire1.write(CTRL_REG4);
  Wire1.write(0x30);

  Wire1.endTransmission();

  return true;
}

void L3G4200D::Read(double &x, double &y, double &z) {
  Wire1.beginTransmission(I2C_ADDRESS);
  Wire1.write(OUT_X_L | 0x80);
  Wire1.endTransmission();

  Wire1.requestFrom(I2C_ADDRESS, 6);

  x = Wire1.read() | (Wire1.read() << 8);
  y = Wire1.read() | (Wire1.read() << 8);
  z = Wire1.read() | (Wire1.read() << 8);

  x = x * scaleFactor;
  y = y * scaleFactor;
  z = z * scaleFactor;

  x = (atan2(y, x) * 180) / PI;
  y = (atan2(z, x) * 180) / PI;
  z = (atan2(z, y) * 180) / PI;
}