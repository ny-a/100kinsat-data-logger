#pragma once

#include "./state.hpp"
#include "MPU9250.h"

class IMU {
  public:
    IMU(State * state);

    void setup();
    void getHeader(String& buffer);
    void selectMadgwickFilter();
    bool update();
    void getLogString(String& buffer);
    void setNorthYaw(double value);
    bool checkValue(int limit);

    MPU9250 mpu;

    double accX = 0.0;
    double accY = 0.0;
    double accZ = 0.0;
    double gyroX = 0.0;
    double gyroY = 0.0;
    double gyroZ = 0.0;
    double magX = 0.0;
    double magY = 0.0;
    double magZ = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;

  private:
    State * state;
};

IMU::IMU(State * state) {
  this->state = state;
}

// setup 内でする必要がありそう
void IMU::setup() {
  Wire.begin();
  delay(1000);

  mpu.setup(0x68);
}

void IMU::getHeader(String& buffer) {
  buffer += String("MPU9250,Yaw,Pitch,Roll,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,MyYaw\n");
}

void IMU::selectMadgwickFilter() {
  mpu.selectFilter(QuatFilterSel::MADGWICK);
}

bool IMU::update() {
  bool isUpdated = mpu.update();
  if (isUpdated) {
    yaw = mpu.getYaw() + 180.0 - state->northYaw;
    if (yaw < 0.0) {
      yaw += 360.0;
    }
    pitch = mpu.getPitch();
    roll = mpu.getRoll();

    accX = mpu.getAccX();
    accY = mpu.getAccY();
    accZ = mpu.getAccZ();
    gyroX = mpu.getGyroX();
    gyroY = mpu.getGyroY();
    gyroZ = mpu.getGyroZ();
    magX = mpu.getMagX();
    magY = mpu.getMagY();
    magZ = mpu.getMagZ();
  }

  return isUpdated;
}

void IMU::getLogString(String& buffer) {
  buffer += String("MPU9250,");

  buffer += String(yaw, 6);
  buffer += String(",");
  buffer += String(pitch, 6);
  buffer += String(",");
  buffer += String(roll, 6);
  buffer += String(",");

  buffer += String(accX, 6);
  buffer += String(",");
  buffer += String(accY, 6);
  buffer += String(",");
  buffer += String(accZ, 6);
  buffer += String(",");
  buffer += String(gyroX, 6);
  buffer += String(",");
  buffer += String(gyroY, 6);
  buffer += String(",");
  buffer += String(gyroZ, 6);
  buffer += String(",");
  buffer += String(magX, 6);
  buffer += String(",");
  buffer += String(magY, 6);
  buffer += String(",");
  buffer += String(magZ, 6);

  buffer += String("\n");
}

bool IMU::checkValue(int limit) {
  unsigned long ms = 100;
  for (int i = 0; i < limit; i++) {
    unsigned long start = millis();
    while (!update() && millis() - start < ms) {}
    if (
      magX != 0.0 ||
      magY != 0.0 ||
      magZ != 0.0
    ) {
      return true;
    }
  }
  return false;
}
