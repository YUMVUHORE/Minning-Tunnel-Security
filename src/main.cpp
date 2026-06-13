#include <HardwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "mpuHandler/mpuHandler.h"
#include "global.h"
String orientation = "Unknown";
HardwareSerial RS485Serial(2);

SensorData sensorData;

// ---------------- NODE CONFIGURATION ----------------
// Set this board's role and address here (or replace with
// EEPROM / DIP-switch reading later).
//
// isMaster = true  -> this board is address 0x00 and polls all slaves
// isMaster = false -> this board is a slave, set MY_ADDR to its id (1-9)
bool isMaster = true;
const int MY_ADDR = 0;
// ------------------------------------------------------

const int MAX_NODES = 10;

struct SlaveData
{
    int id;

    float x;
    float y;
    float z;

    float pitch;
    float roll;
    float angle;

    String orientation;

    unsigned long lastSeen;
};

SlaveData slaves[MAX_NODES];

const unsigned long NODE_TIMEOUT = 3000;   // ms before a node is considered offline
const unsigned long POLL_WAIT    = 100;    // ms to wait for a slave reply

int currentPoll = 1; // master starts polling from slave 0x01

// -------------------- Helpers --------------------

void parsePacket(String msg)
{
    msg.trim();

    int p1 = msg.indexOf(',');
    int p2 = msg.indexOf(',', p1 + 1);
    int p3 = msg.indexOf(',', p2 + 1);
    int p4 = msg.indexOf(',', p3 + 1);
    int p5 = msg.indexOf(',', p4 + 1);
    int p6 = msg.indexOf(',', p5 + 1);
    int p7 = msg.indexOf(',', p6 + 1);

    if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0 || p5 < 0 || p6 < 0 || p7 < 0)
        return;

    int id = msg.substring(0, p1).toInt();

    if (id < 0 || id >= MAX_NODES)
        return;

    slaves[id].id = id;
    slaves[id].x = msg.substring(p1 + 1, p2).toFloat();
    slaves[id].y = msg.substring(p2 + 1, p3).toFloat();
    slaves[id].z = msg.substring(p3 + 1, p4).toFloat();
    slaves[id].pitch = msg.substring(p4 + 1, p5).toFloat();
    slaves[id].roll = msg.substring(p5 + 1, p6).toFloat();
    slaves[id].angle = msg.substring(p6 + 1, p7).toFloat();
    slaves[id].orientation = msg.substring(p7 + 1);
    slaves[id].lastSeen = millis();
}

// Build and send this node's current sensor reading as a CSV packet,
// prefixed by its own address.
void sendSensorPacket(int addr)
{
    RS485Serial.print(addr);
    RS485Serial.print(",");
    RS485Serial.print(sensorData.posX, 3);
    RS485Serial.print(",");
    RS485Serial.print(sensorData.posY, 3);
    RS485Serial.print(",");
    RS485Serial.print(sensorData.posZ, 3);
    RS485Serial.print(",");
    RS485Serial.print(sensorData.pitch, 1);
    RS485Serial.print(",");
    RS485Serial.print(sensorData.roll, 1);
    RS485Serial.print(",");
    RS485Serial.print(sensorData.angleFromVertical, 1);
    RS485Serial.print(",");
    RS485Serial.println(sensorData.orientation);
}

// Store the master's own MPU6050 reading into slaves[0], so the
// display routine can treat master and slaves identically.
void updateOwnData(int addr)
{
    slaves[addr].id = addr;
    slaves[addr].x = sensorData.posX;
    slaves[addr].y = sensorData.posY;
    slaves[addr].z = sensorData.posZ;
    slaves[addr].pitch = sensorData.pitch;
    slaves[addr].roll = sensorData.roll;
    slaves[addr].angle = sensorData.angleFromVertical;
    slaves[addr].orientation = sensorData.orientation;
    slaves[addr].lastSeen = millis();
}

void displayAllNodes()
{
    unsigned long now = millis();

    Serial.println("================ NODE STATUS ================");

    for (int i = 0; i < MAX_NODES; i++)
    {
        if (slaves[i].lastSeen == 0)
            continue; // never seen, skip

        bool online = (now - slaves[i].lastSeen) < NODE_TIMEOUT;

        Serial.print("Node ");
        Serial.print(i);
        Serial.print(i == 0 ? " (Master)" : " (Slave)");
        Serial.println(online ? " [ONLINE]" : " [OFFLINE]");

        Serial.print("  X=");
        Serial.print(slaves[i].x, 3);
        Serial.print("  Y=");
        Serial.print(slaves[i].y, 3);
        Serial.print("  Z=");
        Serial.println(slaves[i].z, 3);

        Serial.print("  Pitch=");
        Serial.print(slaves[i].pitch, 1);
        Serial.print("  Roll=");
        Serial.print(slaves[i].roll, 1);
        Serial.print("  Angle=");
        Serial.println(slaves[i].angle, 1);

        Serial.print("  Orientation=");
        Serial.println(slaves[i].orientation);
    }

    Serial.println("==============================================");
}

// -------------------- Master logic --------------------

void masterLoop()
{
    readMPU6050();
    updateOwnData(0); // master's own data lives at slaves[0]

    // Poll the next slave in round-robin order
    RS485Serial.print("POLL,");
    if (currentPoll < 10) RS485Serial.print("0");
    RS485Serial.println(currentPoll);

    unsigned long start = millis();
    while (millis() - start < POLL_WAIT)
    {
        if (RS485Serial.available())
        {
            String packet = RS485Serial.readStringUntil('\n');
            parsePacket(packet);
            break;
        }
    }

    currentPoll++;
    if (currentPoll >= MAX_NODES) currentPoll = 1;

    displayAllNodes();
}

// -------------------- Slave logic --------------------

void slaveLoop()
{
    readMPU6050();

    if (RS485Serial.available())
    {
        String packet = RS485Serial.readStringUntil('\n');
        packet.trim();

        if (packet.startsWith("POLL,"))
        {
            int addr = packet.substring(5).toInt();

            if (addr == MY_ADDR)
            {
                sendSensorPacket(MY_ADDR);
            }
        }
    }
}

// -------------------- Setup / Loop --------------------

void setup()
{
    Serial.begin(115200);

    RS485Serial.begin(
        9600,
        SERIAL_8N1,
        16, // RX
        17  // TX
    );

    Wire.begin();
    Wire.setClock(100000);
    setupMPU6050();

    delay(1000);
}

void loop()
{
    if (isMaster)
    {
        masterLoop();
    }
    else
    {
        slaveLoop();
    }
}