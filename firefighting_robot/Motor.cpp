#include "Arduino.h"


#include "Motor.h"

Motor::Motor(int enablePin, int dir1Pin, int dir2Pin, int maxPower, int minPWMRange, int maxPWMRange) {
  mEnablePin = enablePin;
  mDir1Pin = dir1Pin;
  mDir2Pin = dir2Pin;
  mMaxPower = maxPower;
  mMinPWMRange = minPWMRange;
  mMaxPWMRange = maxPWMRange;

}

void Motor::setPower(int power) {
  // Sets the motor actuation level (open-loop control).
  // The parameter 'power' is an abstract control input in the range [-maxPower, +maxPower].
  // It does NOT represent physical speed (e.g., m/s or rad/s), since no feedback is used.
  // Instead, it defines the direction and intensity of the voltage applied to the motor:
  // - power = 0  → Motor stops
  // - power < 0  → Motor rotates in reverse
  // - power > 0  → Motor rotates forward

  // Stop the motor if power = 0
  if(power == 0) {
    stop();
    return;
  }

  // Select rotation direction based on the sign of 'power'.
  // Note: depending on how the motor is wired to the driver, the direction may be inverted.
  // In such cases, either swap the control signals (Dir1/Dir2) in software or physically
  // reverse the motor connections to ensure consistent forward/reverse behaviour.
  if(power > 0) {
     digitalWrite(mDir1Pin, LOW);
     digitalWrite(mDir2Pin, HIGH);
  }
  else {
    digitalWrite(mDir1Pin, HIGH);
    digitalWrite(mDir2Pin, LOW);
  }

  // Convert the abstract 'power' value to a PWM duty cycle.
  // The motor speed is controlled via PWM using analogWrite(mEnablePin, v),
  // where v ∈ [0, 255] defines the duty cycle.
  // However, not all PWM values are useful in practice:
  // - Below a minimum PWM range (mMinPWMRange) the motor cannot overcome static friction (dead zone).
  // - Above a maximum PWM range (mMaxPWMRange) the motor may reach excessive speed for the application.
  //
  // The user specifies 'power' in a symmetric range [-maxPower, +maxPower],
  // which is mapped to the PWM interval [mMinPWMRange, mMaxPWMRange].
  //
  // This mapping avoids:
  //  - very low PWM values that do not produce motion,
  //  - excessively high PWM values that may be unsafe or unnecessary.
  analogWrite(mEnablePin, map(abs(power), 1, mMaxPower, mMinPWMRange, mMaxPWMRange));  
}

void Motor::stop() {
  // Stops the motor by disabling the PWM signal (no voltage applied to the motor).
  // Note: this results in a free-run (coast), not an active brake.
  digitalWrite(mEnablePin, LOW);
}

void Motor::brake() {
  // Actively brakes the motor by short-circuiting its terminals through the driver.
  // This creates a resistive torque that quickly reduces the motor speed.
  digitalWrite(mEnablePin, HIGH);
  digitalWrite(mDir1Pin, LOW);
  digitalWrite(mDir2Pin, LOW);
}