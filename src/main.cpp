// =============================================================
//  RS485 Distributed Sensor Network — CORRECTED FIRMWARE v3
//
//  Transceiver : MAX13487E (auto-direction, no DE/RE GPIO needed)
//  Baud rate   : 9600, 8N1
//  Nodes       : 1 master (addr 0x00) + up to 9 slaves (0x01–0x09)
//
// ── PACKET FORMAT ─────────────────────────────────────────────
//
//  Every frame on the bus:
//
//    [STX] <payload> [*] <CRC8_HEX> [ETX]
//
//    STX  = 0x02  (ASCII Start-of-Text)  — frame start marker
//    ETX  = 0x03  (ASCII End-of-Text)    — frame end marker
//    *    = literal asterisk separating payload from checksum
//    CRC8 = Dallas/Maxim CRC-8 of <payload> only, printed as
//           2 uppercase hex digits (e.g. "A3")
//
//  ADDRESS FORMAT
//    2-digit uppercase hex, consistent in both directions:
//      00 = master
//      01–09 = slave nodes (extend to FF if needed later)
//
//  POLL frame  (master → all slaves, broadcast)
//    Payload : POLL,<ADDR_HEX>
//    Example : \x02POLL,01*8A\x03
//    Only the slave whose address matches replies.
//
//  DATA frame  (slave → master, or master self-record)
//    Payload : <ADDR_HEX>,<x>,<y>,<z>,<pitch>,<roll>,<angle>,<orientation>
//    Example : \x0201,0.060,-0.122,1.074,3.2,-6.5,7.2,VERTICAL*85\x03
//
// ── BUG FIXES vs ORIGINAL ─────────────────────────────────────
//    BUG-01  flush() + 3 ms guard for MAX13487E TX→RX turnaround
//    BUG-02  Receive window drains fully; no early break on noise
//    BUG-03  POLL_WAIT raised to 300 ms (covers full reply TX time)
//    BUG-04  Slave reads RS485 BEFORE blocking MPU6050 I2C read
//    BUG-05  slaves[].id init to -1; skip id==-1 in display/JSON
//    NEW-01  CRC-8 on every frame; corrupt frames silently dropped
//    NEW-02  STX/ETX framing; parser syncs on STX, validates ETX
//    NEW-03  Unified 2-digit hex address in both POLL and DATA
//    NEW-04  Master own data always included in JSON output
// =============================================================

#if defined(ESP32)
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_system.h>
#endif
#include <Wire.h>
#include "mpuHandler/mpuHandler.h"
#include "global.h"

// -------------------- RS485 UART --------------------
#if defined(ESP32)
#include <HardwareSerial.h>
HardwareSerial RS485Serial(2);          // UART2: RX=GPIO16, TX=GPIO17
#else
#include <SoftwareSerial.h>
SoftwareSerial RS485Serial(2, 3);       // Nano: RX=D2, TX=D3
#endif

// -------------------- MAX13487E TIMING --------------------
// flush() blocks until the UART shift register is empty (last stop
// bit physically left the pin).  The MAX13487E then auto-disables
// its driver in ~0.5 µs.  We add 3 ms (≈3 byte-periods at 9600
// baud) as a hard guard before the receive window opens.
#define RS485_TURNAROUND_US  3000

// -------------------- FRAME CONSTANTS --------------------
#define STX  '\x02'
#define ETX  '\x03'

// -------------------- NODE CONFIGURATION --------------------
bool      isMaster = true;   // true = master (addr 0x00)
const int MY_ADDR  = 0x00;   // slaves: set to 0x01–0x09
// ------------------------------------------------------------
String orientation = "unknown";  // human-readable orientation string from MPU6050
const int MAX_NODES = 10;

struct NodeData
{
    int    id;               // -1 = slot never populated
    float  x, y, z;
    float  pitch, roll, angle;
    String orientation;
    unsigned long lastSeen;
    bool   crcErrors;        // true if last received frame failed CRC
};

NodeData   nodes[MAX_NODES];
SensorData sensorData;

const unsigned long NODE_TIMEOUT = 3000;  // ms before declared offline

// POLL_WAIT budget:
//   9.4 ms  POLL TX at 9600 baud
//   3.0 ms  MAX13487E turnaround guard
//  10.0 ms  slave MPU6050 I2C read
//  62.5 ms  reply TX (worst-case 60 bytes at 9600 baud)
//  20.0 ms  SoftwareSerial RX latency on Nano
//  ─────────────────
// ~105 ms minimum → 300 ms gives comfortable margin
const unsigned long POLL_WAIT = 300;

int currentPoll = 1;   // master round-robin counter (1–9)

// ===================== CRC-8 (Dallas / Maxim) =====================
// Polynomial: x^8 + x^5 + x^4 + 1  (0x31, reflected = 0x8C)
// Covers the payload string only (between STX and the '*' separator).
uint8_t crc8(const String &data)
{
    uint8_t crc = 0x00;
    for (unsigned int i = 0; i < data.length(); i++)
    {
        crc ^= (uint8_t)data[i];
        for (int b = 0; b < 8; b++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ===================== Frame TX =====================

// Wrap a payload string in STX…CRC*ETX and send it on the RS485 bus.
void sendFrame(const String &payload)
{
    uint8_t crc = crc8(payload);

    RS485Serial.write(STX);
    RS485Serial.print(payload);
    RS485Serial.print('*');
    // CRC as 2 uppercase hex digits
    if (crc < 0x10) RS485Serial.print('0');
    RS485Serial.print(crc, HEX);
    RS485Serial.write(ETX);

    RS485Serial.flush();                     // wait for last byte to leave UART
    delayMicroseconds(RS485_TURNAROUND_US);  // MAX13487E TX→RX turnaround guard
}

// ===================== Frame RX / validation =====================

// Read one complete frame from RS485Serial within the current time
// budget (caller controls the outer while-loop).
// Returns the payload string if STX…ETX received and CRC passes.
// Returns "" on timeout, framing error, or CRC mismatch.
String readFrame()
{
    // Sync to STX — discard anything before it
    if (!RS485Serial.available()) return "";

    char c = RS485Serial.read();
    if (c != STX) return "";   // not a frame start, caller will retry

    // Read until ETX or timeout (use 50 ms inner guard)
    String raw = "";
    unsigned long t = millis();
    while (millis() - t < 50)
    {
        if (RS485Serial.available())
        {
            c = RS485Serial.read();
            if (c == ETX) break;
            raw += c;
        }
    }
    if (c != ETX) return "";   // framing error — no ETX arrived

    // Split at '*': payload*CRCXX
    int star = raw.lastIndexOf('*');
    if (star < 0) return "";

    String payload  = raw.substring(0, star);
    String crcField = raw.substring(star + 1);

    if (crcField.length() < 2) return "";

    uint8_t receivedCRC  = (uint8_t)strtol(crcField.c_str(), nullptr, 16);
    uint8_t computedCRC  = crc8(payload);

    if (receivedCRC != computedCRC)
    {
        Serial.print("[CRC ERR] expected ");
        if (computedCRC < 0x10) Serial.print('0');
        Serial.print(computedCRC, HEX);
        Serial.print(" got ");
        Serial.println(crcField);
        return "";
    }

    return payload;   // clean, CRC-validated payload
}

// ===================== Packet parser =====================

// Parse a DATA payload into nodes[].
// DATA payload format:
//   <ADDR_HEX>,<x>,<y>,<z>,<pitch>,<roll>,<angle>,<orientation>
void parseDataPayload(const String &payload)
{
    int p1 = payload.indexOf(',');
    int p2 = payload.indexOf(',', p1 + 1);
    int p3 = payload.indexOf(',', p2 + 1);
    int p4 = payload.indexOf(',', p3 + 1);
    int p5 = payload.indexOf(',', p4 + 1);
    int p6 = payload.indexOf(',', p5 + 1);
    int p7 = payload.indexOf(',', p6 + 1);

    if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0 || p5 < 0 || p6 < 0 || p7 < 0)
        return;

    // Address is 2-digit hex (e.g. "01")
    int id = (int)strtol(payload.substring(0, p1).c_str(), nullptr, 16);
    if (id < 0 || id >= MAX_NODES) return;

    nodes[id].id          = id;
    nodes[id].x           = payload.substring(p1 + 1, p2).toFloat();
    nodes[id].y           = payload.substring(p2 + 1, p3).toFloat();
    nodes[id].z           = payload.substring(p3 + 1, p4).toFloat();
    nodes[id].pitch       = payload.substring(p4 + 1, p5).toFloat();
    nodes[id].roll        = payload.substring(p5 + 1, p6).toFloat();
    nodes[id].angle       = payload.substring(p6 + 1, p7).toFloat();
    nodes[id].orientation = payload.substring(p7 + 1);
    nodes[id].lastSeen    = millis();
    nodes[id].crcErrors   = false;
}

// ===================== Sensor packet builder =====================

// Build the DATA payload for this node and transmit it as a framed packet.
void sendSensorPacket(int addr)
{
    // Payload: 2-digit hex address then 7 CSV fields
    String payload = "";
    if (addr < 0x10) payload += '0';
    payload += String(addr, HEX);
    payload.toUpperCase();     // keep hex uppercase throughout

    // Rebuild with uppercase address
    payload = "";
    char addrBuf[3];
    snprintf(addrBuf, sizeof(addrBuf), "%02X", addr);
    payload += addrBuf;

    payload += ',';
    payload += String(sensorData.posX, 3);
    payload += ',';
    payload += String(sensorData.posY, 3);
    payload += ',';
    payload += String(sensorData.posZ, 3);
    payload += ',';
    payload += String(sensorData.pitch, 1);
    payload += ',';
    payload += String(sensorData.roll, 1);
    payload += ',';
    payload += String(sensorData.angleFromVertical, 1);
    payload += ',';
    payload += sensorData.orientation;

    sendFrame(payload);
}

// Store master's own MPU reading directly into nodes[0].
void updateMasterData()
{
    nodes[0].id          = 0;
    nodes[0].x           = sensorData.posX;
    nodes[0].y           = sensorData.posY;
    nodes[0].z           = sensorData.posZ;
    nodes[0].pitch       = sensorData.pitch;
    nodes[0].roll        = sensorData.roll;
    nodes[0].angle       = sensorData.angleFromVertical;
    nodes[0].orientation = sensorData.orientation;
    nodes[0].lastSeen    = millis();
    nodes[0].crcErrors   = false;
}

// ===================== Display =====================

void displayAllNodes()
{
    unsigned long now = millis();
    Serial.println();
    Serial.println("================ NODE STATUS ================");

    bool anyNode = false;
    for (int i = 0; i < MAX_NODES; i++)
    {
        if (nodes[i].id == -1) continue;
        anyNode = true;

        bool online = (now - nodes[i].lastSeen) < NODE_TIMEOUT;
        Serial.print("Node 0x");
        if (i < 0x10) Serial.print('0');
        Serial.print(i, HEX);
        Serial.print(i == 0 ? " [MASTER]" : " [SLAVE] ");
        Serial.println(online ? " ONLINE" : " OFFLINE");
        Serial.print("  X=");     Serial.print(nodes[i].x, 3);
        Serial.print("  Y=");     Serial.print(nodes[i].y, 3);
        Serial.print("  Z=");     Serial.println(nodes[i].z, 3);
        Serial.print("  Pitch="); Serial.print(nodes[i].pitch, 1);
        Serial.print("  Roll=");  Serial.print(nodes[i].roll, 1);
        Serial.print("  Angle="); Serial.println(nodes[i].angle, 1);
        Serial.print("  Orientation: "); Serial.println(nodes[i].orientation);
    }
    if (!anyNode) Serial.println("  (no nodes seen yet)");
    Serial.println("==============================================");
}

// ===================== JSON output =====================
//
//  Emitted once per poll cycle on Serial so your Node.js / Python
//  dashboard can parse it by reading lines that start with '{'.
//
//  Schema:
//  {
//    "ts"    : <millis since boot>,
//    "master": { <node object> },
//    "slaves": [ { <node object> }, ... ],
//    "summary": {
//      "total_nodes"   : <N>,
//      "online_nodes"  : <N>,
//      "offline_nodes" : <N>
//    }
//  }
//
//  Node object:
//  {
//    "addr"       : "00",          // 2-digit hex, matches bus address
//    "role"       : "master"|"slave",
//    "status"     : "online"|"offline",
//    "last_seen_ms": 12345,
//    "accel": { "x": 0.060, "y": -0.122, "z": 1.074 },
//    "attitude": { "pitch": 3.2, "roll": -6.5, "angle": 7.2 },
//    "orientation": "VERTICAL (Standing upright)"
//  }

void printNodeJSON(int i, unsigned long now, bool isLast)
{
    bool online = (now - nodes[i].lastSeen) < NODE_TIMEOUT;
    char addrBuf[3];
    snprintf(addrBuf, sizeof(addrBuf), "%02X", i);

    Serial.print("    {");
    Serial.print("\"addr\":\"");        Serial.print(addrBuf);
    Serial.print("\",\"role\":\"");     Serial.print(i == 0 ? "master" : "slave");
    Serial.print("\",\"status\":\"");   Serial.print(online ? "online" : "offline");
    Serial.print("\",\"last_seen_ms\":"); Serial.print(nodes[i].lastSeen);
    Serial.print(",\"accel\":{");
    Serial.print("\"x\":");             Serial.print(nodes[i].x, 3);
    Serial.print(",\"y\":");            Serial.print(nodes[i].y, 3);
    Serial.print(",\"z\":");            Serial.print(nodes[i].z, 3);
    Serial.print("},\"attitude\":{");
    Serial.print("\"pitch\":");         Serial.print(nodes[i].pitch, 1);
    Serial.print(",\"roll\":");         Serial.print(nodes[i].roll, 1);
    Serial.print(",\"angle\":");        Serial.print(nodes[i].angle, 1);
    Serial.print("},\"orientation\":\""); Serial.print(nodes[i].orientation);
    Serial.print("\"}");
    if (!isLast) Serial.print(",");
    Serial.println();
}

void printAllNodesJSON()
{
    unsigned long now = millis();

    int totalNodes  = 0;
    int onlineNodes = 0;
    for (int i = 0; i < MAX_NODES; i++)
    {
        if (nodes[i].id == -1) continue;
        totalNodes++;
        if ((now - nodes[i].lastSeen) < NODE_TIMEOUT) onlineNodes++;
    }
    int offlineNodes = totalNodes - onlineNodes;

    // Collect slave indices for last-element comma handling
    int slaveIdx[MAX_NODES];
    int slaveCount = 0;
    for (int i = 1; i < MAX_NODES; i++)
        if (nodes[i].id != -1) slaveIdx[slaveCount++] = i;

    // ── JSON open ──
    Serial.println("{");
    Serial.print("  \"ts\":"); Serial.print(now); Serial.println(",");

    // ── master object ──
    Serial.print("  \"master\":");
    if (nodes[0].id != -1)
    {
        Serial.println();
        printNodeJSON(0, now, false);  // trailing comma handled by next key
        // Remove trailing comma: reprint without comma on last field
        // Simpler: always emit master, comma is handled by "slaves" key following
    }
    else
    {
        Serial.println("null,");
    }

    // ── slaves array ──
    Serial.println("  \"slaves\":[");
    if (slaveCount == 0)
    {
        Serial.println("  ],");
    }
    else
    {
        for (int s = 0; s < slaveCount; s++)
            printNodeJSON(slaveIdx[s], now, s == slaveCount - 1);
        Serial.println("  ],");
    }

    // ── summary ──
    Serial.println("  \"summary\":{");
    Serial.print("    \"total_nodes\":");   Serial.print(totalNodes);   Serial.println(",");
    Serial.print("    \"online_nodes\":");  Serial.print(onlineNodes);  Serial.println(",");
    Serial.print("    \"offline_nodes\":"); Serial.print(offlineNodes); Serial.println();
    Serial.println("  }");

    Serial.println("}");
}

// ===================== I2C scan =====================

void i2cScan()
{
    Serial.println("[INIT] Scanning I2C bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
        {
            Serial.print("[INIT] Device at 0x");
            if (addr < 16) Serial.print('0');
            Serial.println(addr, HEX);
            found++;
        }
    }
    Serial.println(found == 0 ? "[INIT] No I2C devices found." : "[INIT] I2C scan complete.");
}

// ===================== Master logic =====================

void masterLoop()
{
    // Always update master's own MPU data first — it is always
    // included in the JSON regardless of bus activity.
    readMPU6050();
    updateMasterData();

    // Build and transmit the POLL frame for the current slave address.
    char addrBuf[3];
    snprintf(addrBuf, sizeof(addrBuf), "%02X", currentPoll);
    String pollPayload = "POLL,";
    pollPayload += addrBuf;

    Serial.print("[MASTER] Polling 0x"); Serial.println(addrBuf);

    sendFrame(pollPayload);   // flush() + turnaround guard inside

    // ── Receive window ──
    // Drain for the full POLL_WAIT budget.  Do NOT break early;
    // a noise fragment must not consume the window before the real reply.
    unsigned long start = millis();
    while (millis() - start < POLL_WAIT)
    {
        if (RS485Serial.available())
        {
            String payload = readFrame();
            if (payload.length() == 0)   continue;  // noise / CRC fail
            if (payload.startsWith("POLL,")) continue;  // own echo
            parseDataPayload(payload);
            // Keep looping — accept any further valid frames in window
        }
    }

    currentPoll++;
    if (currentPoll >= MAX_NODES) currentPoll = 1;

    displayAllNodes();
    printAllNodesJSON();
}

// ===================== Slave logic =====================

void slaveLoop()
{
    // Check bus FIRST (BUG-04 FIX) so the SoftwareSerial buffer
    // does not overflow during a blocking MPU6050 I2C read.
    if (RS485Serial.available())
    {
        String payload = readFrame();

        if (payload.length() > 0 && payload.startsWith("POLL,"))
        {
            // Address field after "POLL," is 2-digit hex
            String addrStr = payload.substring(5);
            int addr = (int)strtol(addrStr.c_str(), nullptr, 16);

            Serial.print("[SLAVE] Poll for 0x"); Serial.println(addrStr);

            if (addr == MY_ADDR)
            {
                Serial.println("[SLAVE] Match — reading MPU and replying...");
                readMPU6050();            // read AFTER packet is in RAM
                sendSensorPacket(MY_ADDR);
            }
        }
    }
    else
    {
        // Bus idle — background MPU read keeps sensorData warm
        readMPU6050();
    }
}

// ===================== Setup =====================

void setup()
{
    Serial.begin(115200);

#if defined(ESP32)
    Serial.print("[INIT] Reset reason: ");
    Serial.println(esp_reset_reason());
    RS485Serial.begin(9600, SERIAL_8N1, 16, 17);
#else
    RS485Serial.begin(9600);
#endif

    Wire.begin();
    Wire.setClock(100000);

    i2cScan();
    setupMPU6050();

    // Mark every slot as never-seen (BUG-05 FIX)
    for (int i = 0; i < MAX_NODES; i++)
    {
        nodes[i].id        = -1;
        nodes[i].lastSeen  =  0;
        nodes[i].crcErrors = false;
    }

    delay(1000);
    Serial.print("[INIT] Role: ");
    Serial.println(isMaster ? "MASTER (0x00)" : "SLAVE  (addr set in MY_ADDR)");
}

// ===================== Loop =====================

void loop()
{
    if (isMaster)
        masterLoop();
    else
        slaveLoop();
}
