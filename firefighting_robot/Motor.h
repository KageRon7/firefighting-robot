// Ensures that this header file is included only once per compilation unit.
// It provides the same protection as include guards, but with simpler syntax.
#pragma once

#include <Arduino.h>

class Motor {
  private:
    int mEnablePin;
    int mDir1Pin;
    int mDir2Pin;
    int mMaxPower;
    int mMinPWMRange;
    int mMaxPWMRange;

  public:
    // Constructor: initialises the motor control interface and configuration parameters
    Motor(int enablePin, int dir1Pin, int dir2Pin, int maxPower, int minPWMRange, int maxPWMRange);
    
    // Sets motor actuation level (direction and intensity)
    void setPower(int power);
    
    // Stops the motor (coast mode: no active braking)
    void stop();

    // Applies active braking to quickly stop the motor
    void brake();

};