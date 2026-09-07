#include "FlameSensor.h"
#include <Wire.h>

FlameSensor::FlameSensor(int zones) {
  // Constructor with the 'zones' parameter, allowing the configuration of the number of vertical regions 
  // used to divide the sensor's horizontal field of view for determining the direction of the IR source (see the update function). 
  
  if(zones < 1) zones = 1;
  else if (zones > 5) zones = 5;

  mZoneWidth = 1023 / zones;
}

void FlameSensor::write2Bytes(byte b1, byte b2) {
  // Auxiliary function for writing a byte (b1) to an I2C device register (b2).
  Wire.beginTransmission(ADDRESS);
  Wire.write(b1);
  Wire.write(b2);
  Wire.endTransmission();
}

void FlameSensor::begin() {
  // Function to execute the sequence of commands to initialize the sensor.
  //Turn camera On.
  write2Bytes(0x30, 0x01);  delay(10);
  write2Bytes(0x30, 0x08);  delay(10);
  // Set sensitivity 
  write2Bytes(0x06, 0x90); delay(10);
  write2Bytes(0x08, 0xC0); delay(10);
  write2Bytes(0x1A, 0x40); delay(10);
   // Set mode (Extended mode = sensor returns coordinates + size)
  write2Bytes(0x33, 0x33); delay(10);

  delay(100);
}

void FlameSensor::update() {
  // Function to process the data sent by the sensor and update the flame parameters:
  // mFlameX
  // mFlameY
  // mFlameSize 
  // mFlameDirection

  //Prepare the sensor to send data starting at register 0x36.
  Wire.beginTransmission(ADDRESS);
  Wire.write(0x36);
  Wire.endTransmission();

  // The sensor tracks up to 4 IR sources and stores the data in registers, ordered by source intensity.
  // In Extended Mode, each source provides 3 bytes of data, that gives a total of 12 bytes.
  // We request only 4 bytes, corresponding to the first source (the one with the highest intensity),
  // which is assumed to correspond to the flame.
  // We request 4 bytes instead of 3 because, according to the documentation, the first byte should be ignored.
  Wire.requestFrom(ADDRESS, 4);
  while(Wire.available() < 4);

  byte byte0 = Wire.read();
  byte0 = Wire.read();
  byte byte1 = Wire.read();
  byte byte2 = Wire.read();

   // The location of the IR source is encoded using 10 bits, with a range of 0–1023 for the X coordinate 
  // and 0–767 for the Y coordinate. The size value has a range from 0 to 15.
  // As the X and Y data exceed one byte, their values are spread across three bytes.
  // Therefore, bitwise operations are necessary to correctly extract the X, Y, and size values 
  // (refer to the sensor slides for detailed information).
   mFlameX = byte0 + ((byte2 & 0x30) << 4);
   mFlameY = byte1 + ((byte2 & 0xC0) << 2);
   mFlameSize = byte2 & 0x0F;

  // To facilitate the implementation of algorithms that drive the robot toward the flame, 
  // the flame's X coordinate is converted into a discrete direction value indicating 
  // whether the flame is to the left, center, or right.
  // The direction is determined by dividing the sensor’s horizontal field of view 
  // into a number of equal sections, as specified in the constructor.
  // Each section has a width of mZoneWidth, and the direction is calculated as:
  // mFlameDirection = (mFlameX / mZoneWidth) + 1.
   if(mFlameX < 1023)
     mFlameDirection = mFlameX / mZoneWidth + 1;
   else
     mFlameDirection = 0;
}

int FlameSensor::getFlameX() {
  return mFlameX;
}

int FlameSensor::getFlameY() {
  return mFlameY;
}

int FlameSensor::getFlameSize() {
  return mFlameSize;
}

int FlameSensor::getFlameDirection() {
  return mFlameDirection;
}