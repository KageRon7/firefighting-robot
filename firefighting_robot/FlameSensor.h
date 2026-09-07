// FlameSensor
// Class to use the IR Camera sensor from DFRobot as a flame sensor.

#pragma once
#include <Arduino.h>

#define ADDRESS 0x58

class FlameSensor {
  private:
    int mZoneWidth;
    int mFlameX;
    int mFlameY;
    int mFlameSize;
    int mFlameDirection;
    void write2Bytes(byte b1, byte b2);

  public:
    FlameSensor(int zones);
    void begin();
    void update();
    int getFlameX();
    int getFlameY();
    int getFlameSize();
    int getFlameDirection();
};