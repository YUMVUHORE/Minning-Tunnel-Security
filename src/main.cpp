#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "mpuHandler/mpuHandler.h"
#include "global.h"
// Adafruit_MPU6050 mpu;
SensorData sensorData;
String orientation = "Unknown";
void setup()
{
    Serial.begin(115200);

    Wire.begin();     // IMPORTANT
 Wire.setClock(100000);
    setupMPU6050();

    delay(1000);
}

void loop()
{
    readMPU6050();

    Serial.print("X: ");
    Serial.print(sensorData.posX, 3);

    Serial.print(" g   Y: ");
    Serial.print(sensorData.posY, 3);

    Serial.print(" g   Z: ");
    Serial.print(sensorData.posZ, 3);

    Serial.print(" g   Pitch: ");
    Serial.print(sensorData.pitch, 1);

    Serial.print("   Roll: ");
    Serial.print(sensorData.roll, 1);

    Serial.print("   Angle: ");
    Serial.print(sensorData.angleFromVertical, 1);

    Serial.print("   Orientation: ");
    Serial.println(sensorData.orientation);

    delay(500);
}
