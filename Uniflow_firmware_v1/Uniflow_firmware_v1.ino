/*
 * UNIFLOW Water Quality Monitoring v1.4.6
 * Board: FireBeetle ESP32-E
 * Sensors: DS18B20 (Temp), TDS, pH, Turbidity
 */

#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include "DFRobot_PH.h"

// --- KONFIGURASI WIFI & MQTT ---
const char* primary_ssid     = "TelU-IOT";
const char* primary_password = "IOT@2023";

const char* mqtt_server = "broker.emqx.io";
const int   mqtt_port   = 1883;
const char* mqtt_topic  = "telkom/uniflow-telu";
const char* base_device_id = "UNIFLOW-01";

const char* AP_SSID     = "UniFlow-Setup";
const char* AP_PASSWORD = "";

// --- LAYOUT EEPROM ---
#define EEPROM_SIZE       512
#define EEPROM_SSID_ADDR  200
#define EEPROM_PASS_ADDR  264
#define EEPROM_FIELD_LEN  64

// --- KONFIGURASI PIN ---
#define TDS_PIN           36
#define PH_PIN            34
#define TURBIDITY_PIN     39
#define ONE_WIRE_BUS      14
#define BUZZER_PIN        26

#define VREF              3.3f
#define ADC_MAX           4095.0f
#define SCOUNT            30
#define MAX_NETWORKS      20

// --- VARIABEL GLOBAL ---
WiFiClient        espClient;
PubSubClient      client(espClient);
WebServer         server(80);
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DFRobot_PH        ph;

int analogBufferTds[SCOUNT], analogBufferPh[SCOUNT], analogBufferTur[SCOUNT];
int analogBufferIndex = 0;

float tempVal = 25.0f, tdsVal = 0.0f, phVal = 7.0f, turVal = 0.0f, phVolt = 0.0f;
float lastNTU = 0.0f;

bool phOK = true, tdsOK = true, turOK = true, suhuOK = true, airAman = true;

// State Manajemen Jaringan
bool isAPMode = false, pendingConnect = false, pendingBeginCalled = false;
String pendingSSID = "", pendingPassword = "";
unsigned long pendingConnectStart = 0;
const unsigned long PENDING_CONNECT_TIMEOUT = 10000UL;

int wifiFailCount = 0;
unsigned long lastWifiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 5000UL;
const int WIFI_ATTEMPT_MAX = 20;

// State Buzzer
unsigned long lastBuzzerAlert       = 0;
unsigned long buzzerOnAt            = 0;
bool buzzerActive                   = false;
const unsigned long BUZZER_INTERVAL = 3600000UL; 
const unsigned long BUZZER_DURATION = 30000UL;

// State Server
bool initialScanStarted = false, serverStarted = false;

// --- FUNGSI UTILITAS ---
int getMedian(int bArray[], int iFilterLen) {
  if (iFilterLen > SCOUNT) iFilterLen = SCOUNT;
  int bTab[SCOUNT];
  for (int i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];
  for (int j = 0; j < iFilterLen - 1; j++) {
    for (int i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        int tmp = bTab[i]; bTab[i] = bTab[i + 1]; bTab[i + 1] = tmp;
      }
    }
  }
  if (iFilterLen & 1) return bTab[iFilterLen / 2];
  return (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
}

String readEEPROMString(int addr) {
  String s = "";
  for (int i = addr; i < addr + EEPROM_FIELD_LEN; i++) {
    char c = (char)EEPROM.read(i);
    if (c == 0 || c == (char)255) break;
    s += c;
  }
  return s;
}

void writeEEPROMString(int addr, const String& s) {
  for (int i = 0; i < EEPROM_FIELD_LEN; i++) {
    EEPROM.write(addr + i, (i < (int)s.length()) ? s[i] : 0);
  }
  EEPROM.commit();
}

void addCORSHeaders() {
  server.sendHeader("Access-Control-Allow-Origin",  "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
  server.sendHeader("Access-Control-Max-Age",       "86400");
}

void evaluasiKualitas() {
  phOK   = (phVal >= 6.5f && phVal <= 8.5f);
  tdsOK  = (tdsVal <= 1000.0f);
  turOK  = (turVal <= 25.0f);
  suhuOK = (tempVal >= 25.0f && tempVal <= 31.0f);   
  airAman = phOK && tdsOK && turOK && suhuOK;
}

// --- FUNGSI REST API ---
void handleWifiScan() {
  addCORSHeaders();
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_FAILED || (!initialScanStarted && n <= 0)) {
    WiFi.scanNetworks(true, true);
    initialScanStarted = true;
    server.send(202, "application/json", "{\"networks\":[],\"scanning\":true}");
    return;
  }
  if (n == WIFI_SCAN_RUNNING) {
    server.send(202, "application/json", "{\"networks\":[],\"scanning\":true}");
    return;
  }
  if (n <= 0) {
    WiFi.scanNetworks(true, true);
    server.send(200, "application/json", "{\"networks\":[],\"scanning\":false}");
    return;
  }
  int reportCount = min(n, MAX_NETWORKS);
  DynamicJsonDocument doc(256 + reportCount * 80);
  JsonArray networks = doc.createNestedArray("networks");
  for (int i = 0; i < reportCount; i++) {
    JsonObject net = networks.createNestedObject();
    net["ssid"] = WiFi.SSID(i);
    net["rssi"] = WiFi.RSSI(i);
    net["secured"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
  }
  doc["scanning"] = false;
  doc["total"] = n;
  String response;
  serializeJson(doc, response);
  WiFi.scanDelete();
  WiFi.scanNetworks(true, true);
  server.send(200, "application/json", response);
}

void handleWifiStatus() {
  addCORSHeaders();
  DynamicJsonDocument doc(512);
  bool connected = (WiFi.status() == WL_CONNECTED);
  doc["connected"] = connected;
  doc["ap_mode"]   = isAPMode;
  if (connected) {
    doc["ssid"]   = WiFi.SSID();
    doc["ip"]     = WiFi.localIP().toString();
    doc["signal"] = WiFi.RSSI();
  }
  doc["ap_ip"] = WiFi.softAPIP().toString();
  String response; serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleWifiConnect() {
  addCORSHeaders();
  if (!server.hasArg("plain")) return server.send(400, "application/json", "{\"message\":\"Body kosong\"}");
  DynamicJsonDocument req(256);
  if (deserializeJson(req, server.arg("plain"))) return server.send(400, "application/json", "{\"message\":\"JSON invalid\"}");
  
  String ssid = req["ssid"] | "", password = req["password"] | "";
  if (ssid.isEmpty()) return server.send(400, "application/json", "{\"message\":\"SSID kosong\"}");
  
  writeEEPROMString(EEPROM_SSID_ADDR, ssid);
  writeEEPROMString(EEPROM_PASS_ADDR, password);
  pendingSSID = ssid; pendingPassword = password;
  pendingConnect = true; pendingBeginCalled = false;
  pendingConnectStart = millis();
  
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Menghubungkan...\"}");
}

void handleWifiDisconnect() {
  addCORSHeaders();
  WiFi.disconnect(false);
  server.send(200, "application/json", "{\"success\":true}");
}

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    addCORSHeaders(); server.send(200, "application/json", "{\"device\":\"UNIFLOW\",\"version\":\"1.4.6\"}");
  });
  server.on("/api/wifi/scan", HTTP_GET, handleWifiScan);
  server.on("/api/wifi/status", HTTP_GET, handleWifiStatus);
  server.on("/api/wifi/connect", HTTP_POST, handleWifiConnect);
  server.on("/api/wifi/disconnect", HTTP_POST, handleWifiDisconnect);
  server.onNotFound([]() { addCORSHeaders(); server.send(404, "application/json", "{\"error\":\"Not found\"}"); });
  server.begin();
}

// --- MANAJEMEN KONEKSI ---
bool connectPrimaryWifi() {
  Serial.printf("\nMencoba WiFi Utama: %s\n", primary_ssid);
  WiFi.mode(WIFI_STA); WiFi.disconnect(true); delay(100);
  WiFi.begin(primary_ssid, primary_password);
  
  for (int i = 0; i < WIFI_ATTEMPT_MAX && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
  }
  return WiFi.status() == WL_CONNECTED;
}

bool connectSavedWifi() {
  String savedSSID = readEEPROMString(EEPROM_SSID_ADDR);
  String savedPass = readEEPROMString(EEPROM_PASS_ADDR);
  if (savedSSID.isEmpty()) return false;
  
  Serial.printf("\nMencoba WiFi Tersimpan: %s\n", savedSSID.c_str());
  WiFi.mode(WIFI_STA); WiFi.disconnect(true); delay(100);
  WiFi.begin(savedSSID.c_str(), savedPass.c_str());
  
  for (int i = 0; i < WIFI_ATTEMPT_MAX && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
  }
  return WiFi.status() == WL_CONNECTED;
}

void startAPMode() {
  Serial.println("\nBERPINDAH KE AP MODE...");
  WiFi.mode(WIFI_AP_STA); WiFi.disconnect(true); delay(100);
  WiFi.softAP(AP_SSID, AP_PASSWORD); WiFi.setSleep(false);
  isAPMode = true;
  
  if (!serverStarted) { setupWebServer(); serverStarted = true; }
  WiFi.scanNetworks(true, true); initialScanStarted = true;
}

void connectWiFi() {
  if (connectPrimaryWifi() || connectSavedWifi()) {
    Serial.printf("\n[OK] TERHUBUNG! IP: %s\n", WiFi.localIP().toString().c_str());
    isAPMode = false;
  } else {
    startAPMode();
  }
}

void reconnectMQTT() {
  if (isAPMode) return;
  String clientId = String(base_device_id) + "-" + WiFi.macAddress().substring(9);
  clientId.replace(":", ""); 
  
  int attempts = 0;
  while (!client.connected() && attempts < 3) {
    if (client.connect(clientId.c_str())) Serial.println("[MQTT] Terhubung");
    else { delay(2000); attempts++; }
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200); delay(200);

  // Cek Data EEPROM 
  EEPROM.begin(EEPROM_SIZE);
  float checkVal = 0.0f; EEPROM.get(0, checkVal);
  if (isnan(checkVal) || isinf(checkVal) || checkVal < 0.001f || checkVal > 999.0f) {
    for (int i = 0; i < 200; i++) EEPROM.write(i, 0xFF);
    EEPROM.commit();
  }

  sensors.begin(); ph.begin();
  pinMode(TDS_PIN, INPUT); pinMode(PH_PIN, INPUT); pinMode(TURBIDITY_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

  connectWiFi();
  if (!serverStarted) { setupWebServer(); serverStarted = true; }

  client.setBufferSize(512);
  if (!isAPMode) client.setServer(mqtt_server, mqtt_port);

  lastBuzzerAlert = millis();
}

// --- MAIN LOOP ---
void loop() {
  server.handleClient();

  // 1. Logika WiFi Connect
  if (pendingConnect) {
    if (WiFi.status() == WL_CONNECTED) {
      pendingConnect = pendingBeginCalled = false; isAPMode = false;
      client.setServer(mqtt_server, mqtt_port);
    } else if (millis() - pendingConnectStart < PENDING_CONNECT_TIMEOUT) {
      if (!pendingBeginCalled) {
        WiFi.mode(WIFI_STA); WiFi.begin(pendingSSID.c_str(), pendingPassword.c_str());
        pendingBeginCalled = true;
      }
    } else {
      pendingConnect = pendingBeginCalled = false;
      WiFi.disconnect(false); delay(100);
      WiFi.softAP(AP_SSID, AP_PASSWORD); WiFi.setSleep(false);
    }
  }

  // 2. Auto Reconnect WiFi
  unsigned long now = millis();
  if (!isAPMode && !pendingConnect && (now - lastWifiCheck >= WIFI_CHECK_INTERVAL)) {
    lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      if (!connectPrimaryWifi() && !connectSavedWifi()) startAPMode();
      else { isAPMode = false; client.setServer(mqtt_server, mqtt_port); }
    }
  }

  // 3. Keep-alive MQTT
  if (!isAPMode) {
    if (!client.connected()) reconnectMQTT();
    client.loop();
  }

  // 4. Timer Buzzer
  if (!buzzerActive && (now - lastBuzzerAlert >= BUZZER_INTERVAL)) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerActive = true;
    buzzerOnAt   = now;
  }
  if (buzzerActive && (now - buzzerOnAt >= BUZZER_DURATION)) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive    = false;
    lastBuzzerAlert = now;
  }

  // 5. Sampling Sensor
  static unsigned long lastSample = 0;
  if (millis() - lastSample > 40) {
    lastSample = millis();
    analogBufferTds[analogBufferIndex] = analogRead(TDS_PIN);
    analogBufferPh[analogBufferIndex]  = analogRead(PH_PIN);
    analogBufferTur[analogBufferIndex] = analogRead(TURBIDITY_PIN);
    analogBufferIndex = (analogBufferIndex + 1) % SCOUNT;
  }

  // 6. Kalkulasi & Publish Data
  if (!isAPMode) {
    static unsigned long lastPublish = 0;
    if (millis() - lastPublish > 2000) {
      lastPublish = millis();

      // --- Suhu ---
      sensors.requestTemperatures();
      float t = sensors.getTempCByIndex(0);
      if (t != DEVICE_DISCONNECTED_C) tempVal = t;

      // --- TDS ---
      float vTds = getMedian(analogBufferTds, SCOUNT) * VREF / ADC_MAX;
      float compCoeff = 1.0f + 0.02f * (tempVal - 25.0f);
      float vTdsComp  = vTds / compCoeff;
      tdsVal = (133.42f * pow(vTdsComp, 3) - 255.86f * pow(vTdsComp, 2) + 857.39f * vTdsComp) * 0.5f;
      if (vTds < 0.05f) tdsVal = 0.0f;

      // --- pH ---
      phVolt = getMedian(analogBufferPh, SCOUNT) * VREF / ADC_MAX * 1000.0f;
      phVal  = ph.readPH(phVolt, tempVal);

      // --- Turbidity ---
      int avgTurADC = getMedian(analogBufferTur, SCOUNT);
      float voltageTur = avgTurADC * VREF / ADC_MAX;
      float ntuRaw = (-28.6f * voltageTur) + 96.0f;
      if (ntuRaw < 0.0f) ntuRaw = 0.0f;
      turVal  = (0.2f * ntuRaw) + (0.8f * lastNTU);
      lastNTU = turVal;

      evaluasiKualitas();

      // --- Payload MQTT ---
      StaticJsonDocument<350> doc;
      doc["device_code"]  = base_device_id;
      doc["ph"]           = serialized(String(phVal, 1));
      doc["turbidity"]    = serialized(String(turVal, 1));
      doc["tds"]          = (int)tdsVal;
      doc["temperature"]  = serialized(String(tempVal, 1));
      doc["ph_ok"]        = phOK;
      doc["tds_ok"]       = tdsOK;
      doc["turbidity_ok"] = turOK;
      doc["suhu_ok"]      = suhuOK;
      doc["air_aman"]     = airAman;

      char buffer[350]; serializeJson(doc, buffer);
      
      if(client.connected()) client.publish(mqtt_topic, buffer);
      
      Serial.printf("TEMP: %.1f C | pH: %.2f | TDS: %.0f mg/L | NTU: %.1f | Status: %s\n", 
        tempVal, phVal, tdsVal, turVal, (airAman ? "Aman" : "TIDAK AMAN"));
    }
  }

  // 7. Kalibrasi pH via Serial
  if (!isAPMode && Serial.available()) ph.calibration(phVolt, tempVal);
}