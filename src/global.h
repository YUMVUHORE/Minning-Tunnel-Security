#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>

struct SensorData {
  float posX, posY, posZ;
  float pitch, roll, angleFromVertical;
  String orientation;
  bool mpuValid;
  int mpuInitProgress;
};

extern SensorData sensorData;
extern String orientation;

#endif