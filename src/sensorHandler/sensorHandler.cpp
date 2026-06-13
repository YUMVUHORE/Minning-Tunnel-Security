#include "sensorHandler.h"
#include "../global.h"
#include <Arduino.h>
#include <math.h>

void initializeSensorData()
{
    sensorData.posX = 0;
    sensorData.posY = 0;
    sensorData.posZ = 0;

    sensorData.pitch = 0;
    sensorData.roll = 0;
    sensorData.angleFromVertical = 0;

    if (!sensorData.mpuValid)
    {
        sensorData.orientation = "MPU not found";
    }
    else
    {
        sensorData.orientation = "Ready";
    }
}

static const float WALK_ACCEL_THRESH = 0.18f;
static const float STATIONARY_ACCEL  = 0.07f;

static const float STATIONARY_ANGLE  = 28.0f;
static const float TILTED_ANGLE      = 52.0f;

static const int WALK_CONFIRM_COUNT  = 4;
static const int STILL_CONFIRM_COUNT = 6;

void updateSensorData()
{
    if (!sensorData.mpuValid)
    {
        sensorData.orientation = "Sensor Initializing";
        return;
    }

    static float lastAccelX = 0.0f;
    static float lastAccelY = 0.0f;
    static float lastAccelZ = 0.0f;

    static int walkCount = 0;
    static int stillCount = 0;

    float dx = sensorData.posX - lastAccelX;
    float dy = sensorData.posY - lastAccelY;
    float dz = sensorData.posZ - lastAccelZ;

    float accelChange = sqrt(dx * dx + dy * dy + dz * dz);

    bool isUpright = sensorData.angleFromVertical <= STATIONARY_ANGLE;
    bool isTilted  = sensorData.angleFromVertical > TILTED_ANGLE;

    bool hasMotion = accelChange > WALK_ACCEL_THRESH;
    bool isStill   = accelChange < STATIONARY_ACCEL;

    if (hasMotion && sensorData.angleFromVertical < 55.0f)
    {
        walkCount++;
        stillCount = 0;

        if (walkCount >= WALK_CONFIRM_COUNT)
        {
            sensorData.orientation = "Walking";
        }
    }
    else if (isStill && isUpright)
    {
        stillCount++;
        walkCount = 0;

        if (stillCount >= STILL_CONFIRM_COUNT)
        {
            sensorData.orientation = "Stationary";
        }
    }
    else
    {
        walkCount = 0;
        stillCount = 0;

        if (isTilted)
        {
            sensorData.orientation = "Crutch Tilted";
        }
        else if (sensorData.angleFromVertical > STATIONARY_ANGLE)
        {
            sensorData.orientation = "Active";
        }
        else
        {
            sensorData.orientation = "Stationary";
        }
    }

    lastAccelX = sensorData.posX;
    lastAccelY = sensorData.posY;
    lastAccelZ = sensorData.posZ;
}