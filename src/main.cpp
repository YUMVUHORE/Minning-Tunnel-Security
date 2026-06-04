

/**
 * ============================================================
 * MINING TUNNEL SAFETY SYSTEM — ESP32 MASTER FIRMWARE
 * Hardware: ESP32 + MAX485 + Wi-Fi Access Point
 * ============================================================
 *
 * Reads RS485 frames from all Arduino nodes (up to 10).
 * Hosts a Wi-Fi hotspot (AP mode). Supervisor connects phone/laptop.
 * Opens browser to http://192.168.4.1 — live dashboard auto-refreshes.
 *
 * Wiring:
 *   MAX485 RO  -> GPIO16 (RX2)
 *   MAX485 DI  -> GPIO17 (TX2)  [not used — master only listens]
 *   MAX485 DE  -> GPIO4
 *   MAX485 RE  -> GPIO4  (tie DE+RE together → LOW = RX mode)
 *   RJ45 Pin1  -> A (RS485+)
 *   RJ45 Pin2  -> B (RS485-)
 *
 * AP Credentials: SSID "TunnelSafety" / Password "tunnel1234"
 * Dashboard URL : http://192.168.4.1
 * ============================================================
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

// ---- RS485 Config ----
#define RS485_RX     16
#define RS485_TX     17
#define DE_RE_PIN    4
#define RS485_BAUD   9600

// ---- Wi-Fi AP ----
const char* AP_SSID = "TunnelSafety";
const char* AP_PASS = "tunnel1234";

// ---- Frame protocol ----
#define FRAME_START  0xAA
#define FRAME_END    0x55
#define FRAME_LEN    11     // START + 9 payload bytes + END
#define MAX_NODES    10

// ---- Per-node data structure ----
struct NodeData {
  uint8_t  id;
  int16_t  ax, ay, az;
  bool     alert;
  uint32_t lastSeen;   // millis() timestamp
  bool     active;
};

NodeData nodes[MAX_NODES + 1];   // index 0 unused, 1–10

WebServer server(80);
DNSServer dnsServer;

// ---- Checksum verify ----
uint8_t calcChecksum(uint8_t *buf, uint8_t len) {
  uint8_t csum = 0;
  for (uint8_t i = 0; i < len; i++) csum ^= buf[i];
  return csum;
}

// ---- Parse incoming RS485 frame ----
void parseFrame(uint8_t *buf) {
  // buf[0]=START, buf[1..9]=payload, buf[10]=END
  if (buf[0] != FRAME_START || buf[10] != FRAME_END) return;

  uint8_t *payload = &buf[1];
  uint8_t rxCsum   = payload[8];
  uint8_t calcCsum = calcChecksum(payload, 8);
  if (rxCsum != calcCsum) return;   // checksum mismatch — discard

  uint8_t nodeId = payload[0];
  if (nodeId < 1 || nodeId > MAX_NODES) return;

  nodes[nodeId].id       = nodeId;
  nodes[nodeId].ax       = (int16_t)((payload[1] << 8) | payload[2]);
  nodes[nodeId].ay       = (int16_t)((payload[3] << 8) | payload[4]);
  nodes[nodeId].az       = (int16_t)((payload[5] << 8) | payload[6]);
  nodes[nodeId].alert    = (payload[7] == 0x01);
  nodes[nodeId].lastSeen = millis();
  nodes[nodeId].active   = true;
}

// ---- Read RS485 with timeout ----
void readRS485() {
  static uint8_t buf[FRAME_LEN];
  static uint8_t pos = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (pos == 0 && b != FRAME_START) continue;  // hunt for start

    buf[pos++] = b;

    if (pos == FRAME_LEN) {
      parseFrame(buf);
      pos = 0;
    }
  }
}

// ---- Convert raw to g (±2g, 16384 LSB/g) ----
float toG(int16_t raw) {
  return (float)raw / 16384.0f;
}

// ---- Dashboard HTML ----
String buildDashboard() {
  // Count active nodes and check for alerts
  int activeCount = 0;
  bool anyAlert = false;
  for (int i = 1; i <= MAX_NODES; i++) {
    // Mark stale nodes as inactive
    if (nodes[i].active && ((millis() - nodes[i].lastSeen) > 5000)) {
      nodes[i].active = false;
    }
    if (nodes[i].active) {
      activeCount++;
      if (nodes[i].alert) anyAlert = true;
    }
  }

  String html = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta http-equiv="refresh" content="2">
<title>Tunnel Safety Monitor</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  
  body {
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
    background: linear-gradient(135deg, #0f2027 0%, #203a43 50%, #2c5364 100%);
    color: #f0f0f0;
    padding: 20px;
    min-height: 100vh;
  }
  
  .container { max-width: 1400px; margin: 0 auto; }
  
  .header {
    text-align: center;
    margin-bottom: 28px;
    background: rgba(255,255,255,0.05);
    padding: 24px;
    border-radius: 12px;
    border-left: 4px solid #00bcd4;
    backdrop-filter: blur(10px);
  }
  
  h1 {
    font-size: 2.2em;
    margin-bottom: 8px;
    letter-spacing: 1px;
    font-weight: 700;
    background: linear-gradient(135deg, #00bcd4, #64b5f6);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    background-clip: text;
  }
  
  .sub {
    font-size: 0.95em;
    color: #b3e5fc;
    margin-bottom: 12px;
    font-weight: 500;
  }
  
  .status-bar {
    display: flex;
    justify-content: center;
    gap: 20px;
    flex-wrap: wrap;
    font-size: 0.9em;
  }
  
  .status-item {
    background: rgba(255,255,255,0.08);
    padding: 8px 16px;
    border-radius: 6px;
    display: flex;
    align-items: center;
    gap: 8px;
  }
  
  .status-item strong { color: #00bcd4; }
  
  .banner-alert {
    background: linear-gradient(135deg, #d32f2f 0%, #f44336 100%);
    padding: 18px;
    border-radius: 10px;
    text-align: center;
    font-weight: bold;
    font-size: 1.15em;
    margin-bottom: 20px;
    animation: pulse-alert 1.5s ease-in-out infinite;
    box-shadow: 0 8px 24px rgba(212, 47, 47, 0.4);
    border: 2px solid #ff5252;
  }
  
  .banner-ok {
    background: linear-gradient(135deg, #1b5e20 0%, #2e7d32 100%);
    padding: 18px;
    border-radius: 10px;
    text-align: center;
    font-weight: bold;
    font-size: 1.15em;
    margin-bottom: 20px;
    box-shadow: 0 8px 24px rgba(27, 94, 32, 0.3);
    border: 2px solid #4caf50;
  }
  
  @keyframes pulse-alert {
    0%, 100% { box-shadow: 0 8px 24px rgba(212, 47, 47, 0.4); }
    50% { box-shadow: 0 8px 40px rgba(212, 47, 47, 0.7); }
  }
  
  .empty-state {
    text-align: center;
    padding: 40px;
    background: rgba(255,255,255,0.05);
    border-radius: 10px;
    color: #90caf9;
    font-size: 1.1em;
  }
  
  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(280px, 1fr));
    gap: 16px;
    margin-bottom: 20px;
  }
  
  .card {
    background: linear-gradient(135deg, rgba(30, 60, 110, 0.9) 0%, rgba(33, 58, 97, 0.9) 100%);
    border-radius: 12px;
    padding: 18px;
    border: 2px solid rgba(0, 188, 212, 0.3);
    transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
    backdrop-filter: blur(5px);
  }
  
  .card:hover {
    transform: translateY(-4px);
    box-shadow: 0 8px 24px rgba(0, 188, 212, 0.2);
    border-color: rgba(0, 188, 212, 0.6);
  }
  
  .card.alert-card {
    background: linear-gradient(135deg, rgba(211, 47, 47, 0.15) 0%, rgba(230, 74, 25, 0.15) 100%);
    border-color: rgba(244, 67, 54, 0.6);
  }
  
  .card.alert-card:hover {
    box-shadow: 0 8px 24px rgba(244, 67, 54, 0.3);
    border-color: rgba(244, 67, 54, 0.9);
  }
  
  .card-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 12px;
    border-bottom: 2px solid rgba(0, 188, 212, 0.2);
    padding-bottom: 10px;
  }
  
  .card h3 {
    font-size: 1.1em;
    font-weight: 600;
    color: #00bcd4;
    margin: 0;
  }
  
  .card.alert-card h3 { color: #ff7043; }
  
  .node-position {
    font-size: 0.8em;
    color: #80deea;
    font-weight: 500;
  }
  
  .card.alert-card .node-position { color: #ffab91; }
  
  .metrics {
    display: flex;
    flex-direction: column;
    gap: 10px;
  }
  
  .metric-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 8px;
    background: rgba(255, 255, 255, 0.03);
    border-radius: 6px;
    border-left: 3px solid #00bcd4;
  }
  
  .metric-label {
    font-size: 0.9em;
    color: #b3e5fc;
    font-weight: 500;
  }
  
  .metric-value {
    font-weight: 700;
    color: #4dd0e1;
    font-size: 1.05em;
    font-family: 'Courier New', monospace;
  }
  
  .status-badge {
    margin-top: 12px;
    padding: 10px;
    border-radius: 6px;
    text-align: center;
    font-weight: bold;
    font-size: 0.9em;
  }
  
  .status-ok {
    background: rgba(76, 175, 80, 0.25);
    color: #81c784;
    border: 1px solid #4caf50;
  }
  
  .status-alert {
    background: rgba(244, 67, 54, 0.25);
    color: #ef5350;
    border: 1px solid #f44336;
    animation: pulse-status 1s ease-in-out infinite;
  }
  
  @keyframes pulse-status {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.7; }
  }
  
  footer {
    text-align: center;
    font-size: 0.85em;
    color: #607d8b;
    margin-top: 24px;
    padding-top: 16px;
    border-top: 1px solid rgba(255,255,255,0.1);
  }
</style>
</head>
<body>
<div class="container">
  <div class="header">
    <h1>⛏ Mining Tunnel Safety System</h1>
    <p class="sub">Real-time Structural Monitoring Dashboard</p>
    <div class="status-bar">
      <div class="status-item">
        <span>🌍 Server:</span>
        <strong>)rawhtml";

  html += WiFi.softAPIP().toString();
  html += R"rawhtml(</strong>
      </div>
      <div class="status-item">
        <span>📡 Active Sensors:</span>
        <strong>)rawhtml";
  
  html += String(activeCount);
  html += R"rawhtml(</strong>
      </div>
      <div class="status-item">
        <span>⏱ Refresh:</span>
        <strong>2s</strong>
      </div>
    </div>
  </div>

)rawhtml";

  // Alert banner
  if (anyAlert) {
    html += R"(<div class="banner-alert">⚠ ALERT: Movement Detected! Check Tunnel Immediately</div>)";
  } else if (activeCount > 0) {
    html += R"(<div class="banner-ok">✅ All Systems Normal — Tunnel Conditions Safe</div>)";
  }

  // Show cards only for active nodes with data
  bool hasActiveNodes = false;
  html += R"(<div class="grid">)";

  for (int i = 1; i <= MAX_NODES; i++) {
    if (!nodes[i].active) continue;  // Skip inactive nodes completely
    
    hasActiveNodes = true;
    String cls = "card";
    if (nodes[i].alert) cls += " alert-card";

    html += "<div class='" + cls + "'>";
    html += "<div class='card-header'>";
    html += "<h3>Sensor " + String(i) + "</h3>";
    html += "<span class='node-position'>" + String(i * 10) + "m</span>";
    html += "</div>";

    html += "<div class='metrics'>";
    html += "<div class='metric-row'>";
    html += "<span class='metric-label'>Accel X</span>";
    html += "<span class='metric-value'>" + String(toG(nodes[i].ax), 2) + " g</span>";
    html += "</div>";

    html += "<div class='metric-row'>";
    html += "<span class='metric-label'>Accel Y</span>";
    html += "<span class='metric-value'>" + String(toG(nodes[i].ay), 2) + " g</span>";
    html += "</div>";

    html += "<div class='metric-row'>";
    html += "<span class='metric-label'>Accel Z</span>";
    html += "<span class='metric-value'>" + String(toG(nodes[i].az), 2) + " g</span>";
    html += "</div>";
    html += "</div>";

    if (nodes[i].alert) {
      html += "<div class='status-badge status-alert'>⚠ MOVEMENT DETECTED</div>";
    } else {
      html += "<div class='status-badge status-ok'>✓ Normal</div>";
    }

    html += "</div>";  // card
  }

  html += "</div>";  // grid

  // Show empty state if no active nodes
  if (!hasActiveNodes) {
    html += R"(<div class="empty-state">📡 Waiting for sensor data... Check connections.</div>)";
  }

  html += R"(<footer>IPRC KIGALI — Mining Tunnel Safety System v1.0 | Mechatronics Engineering</footer>
</div>
</body>
</html>)";
  
  return html;

  return html;
}

// ---- Web server handlers ----
void handleRoot()  { server.send(200, "text/html", buildDashboard()); }
void handleNotFound() { server.sendHeader("Location", "http://192.168.4.1"); server.send(302); }

// ---- API endpoint for JSON data ----
void handleAPI() {
  String json = "{\"nodes\":[";
  bool first = true;
  for (int i = 1; i <= MAX_NODES; i++) {
    if (!nodes[i].active) continue;
    if (!first) json += ",";
    json += "{\"id\":" + String(i);
    json += ",\"pos\":" + String(i * 10);
    json += ",\"ax\":"  + String(toG(nodes[i].ax), 3);
    json += ",\"ay\":"  + String(toG(nodes[i].ay), 3);
    json += ",\"az\":"  + String(toG(nodes[i].az), 3);
    json += ",\"alert\":" + String(nodes[i].alert ? "true" : "false");
    json += "}";
    first = false;
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);                            // Debug
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);

  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);   // Always RX mode (master only receives)

  // Clear node table
  for (int i = 0; i <= MAX_NODES; i++) {
    nodes[i] = {0, 0, 0, 0, false, 0, false};
  }

  // Start Wi-Fi AP
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.println("AP started: " + WiFi.softAPIP().toString());

  // Captive DNS — redirect all domains to our IP
  dnsServer.start(53, "*", WiFi.softAPIP());

  // Register routes
  server.on("/",        handleRoot);
  server.on("/api",     handleAPI);
  server.onNotFound(    handleNotFound);
  server.begin();

  Serial.println("Web server started.");
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  readRS485();
}
