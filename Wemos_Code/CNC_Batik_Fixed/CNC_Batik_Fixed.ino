/*
 * CNC Batik Controller - FIXED VERSION
 * Wemos D1 R32 - Sequential Batch Processing
 * 
 * PERBAIKAN:
 * - Auto-start setiap batch yang diterima
 * - Clear buffer setelah selesai proses batch
 * - Kirim status "completed" untuk trigger batch berikutnya
 * - Frontend menunggu "completed" sebelum kirim batch berikutnya
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <vector>

// ===== WiFi Configuration =====
const char* ssid = "DESK";
const char* password = "katakata123";

// ===== MQTT Configuration =====
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_client_id = "wemos_cnc_batik";

// MQTT Topics
const char* topic_gcode = "cnc/batik/gcode";
const char* topic_status = "cnc/batik/status";
const char* topic_control = "cnc/batik/control";
const char* topic_config = "cnc/batik/config";
const char* topic_progress = "cnc/batik/progress";

// ===== Pin Configuration =====
#define STEP_X 26
#define DIR_X 16
#define ENABLE_X 12
#define STEP_Y 25
#define DIR_Y 27
#define ENABLE_Y 12
#define SERVO_PIN 14

// ===== CNC Parameters =====
#define STEPS_PER_MM 40
#define MAX_SPEED 3000
#define MIN_SPEED 100
#define SERVO_PEN_UP 90
#define SERVO_PEN_DOWN 45
#define MAX_X 297.0
#define MAX_Y 210.0
#define MAX_GCODE_BUFFER 600
#define PROGRESS_UPDATE_INTERVAL 10

// ===== Global Variables =====
WiFiClient espClient;
PubSubClient client(espClient);
Servo servoZ;

float currentX = 0.0;
float currentY = 0.0;
float currentZ = SERVO_PEN_UP;

bool isRunning = false;
bool isPaused = false;
bool motorsEnabled = false;
int currentLine = 0;
int totalLines = 0;

bool absoluteMode = true;
bool invertX = false;
bool invertY = false;
bool invertZ = false;

unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_INTERVAL = 1000;

// ===== BATCH G-CODE BUFFER =====
std::vector<String> gcodeBuffer;
bool batchReceived = false;
int batchStartLine = 0;
unsigned long batchReceiveTime = 0;
bool autoStartBatch = false; // ✅ FIXED: Auto-start setiap batch

// ===== Function Prototypes =====
void setup_wifi();
void reconnect_mqtt();
void callback(char* topic, byte* payload, unsigned int length);
void processGcode(String gcode);
void processBatchGcode();
void moveToPosition(float x, float y, int feedRate);
void moveRelative(float dx, float dy, int feedRate);
void setServoZ(float zPos, bool fromGcode = false);
void homeAll();
void enableMotors(bool enable);
void sendStatus(String status);
void sendPositionUpdate();
void sendProgressUpdate(int line, int total);
float parseCoordinate(String gcode, char axis);
bool hasCoordinate(String gcode, char axis);
void emergencyStop();
void clearGcodeBuffer();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   CNC Batik - FIXED VERSION           ║");
  Serial.println("║   Auto-start after ALL batches        ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  pinMode(STEP_X, OUTPUT);
  pinMode(DIR_X, OUTPUT);
  pinMode(ENABLE_X, OUTPUT);
  pinMode(STEP_Y, OUTPUT);
  pinMode(DIR_Y, OUTPUT);
  pinMode(ENABLE_Y, OUTPUT);
  
  digitalWrite(ENABLE_X, HIGH);
  digitalWrite(ENABLE_Y, HIGH);
  motorsEnabled = false;
  
  servoZ.attach(SERVO_PIN);
  servoZ.write(SERVO_PEN_UP);
  currentZ = SERVO_PEN_UP;
  delay(500);
  
  Serial.println("✓ Hardware initialized");
  gcodeBuffer.reserve(MAX_GCODE_BUFFER);
  Serial.println("✓ G-code buffer reserved: " + String(MAX_GCODE_BUFFER) + " lines");
  
  setup_wifi();
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setBufferSize(16384);
  
  Serial.println("✓ MQTT buffer: 16KB");
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║      Setup Complete - Ready!           ║");
  Serial.println("╚════════════════════════════════════════╝\n");
}

void loop() {
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();
  
  // ✅ FIXED: AUTO-START setiap batch diterima
  if (autoStartBatch && batchReceived && !isRunning) {
    Serial.println("\n🚀 AUTO-STARTING BATCH...");
    Serial.println("   autoStartBatch: " + String(autoStartBatch));
    Serial.println("   batchReceived: " + String(batchReceived));
    Serial.println("   isRunning: " + String(isRunning));
    Serial.println("   totalLines: " + String(totalLines));
    
    isRunning = true;
    isPaused = false;
    currentLine = 0;
    enableMotors(true);
    sendStatus("started");
    autoStartBatch = false;
  }
  
  if (batchReceived && isRunning && !isPaused) {
    processBatchGcode();
  }
  
  if (millis() - lastStatusUpdate > STATUS_INTERVAL) {
    sendPositionUpdate();
    lastStatusUpdate = millis();
  }
  
  delay(5);
}

void setup_wifi() {
  delay(10);
  Serial.println("─────────────────────────────────────────");
  Serial.print("📡 Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 30) {
    delay(500);
    Serial.print(".");
    attempt++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✓ WiFi connected!");
    Serial.print("   IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi failed!");
  }
  Serial.println("─────────────────────────────────────────");
}

void reconnect_mqtt() {
  int attempts = 0;
  while (!client.connected() && attempts < 3) {
    Serial.print("📡 MQTT connecting... ");
    
    if (client.connect(mqtt_client_id)) {
      Serial.println("✓ Connected!");
      client.subscribe(topic_gcode);
      client.subscribe(topic_control);
      client.subscribe(topic_config);
      Serial.println("✓ Subscribed to topics");
      sendStatus("online");
      sendPositionUpdate();
    } else {
      Serial.print("✗ Failed, rc=");
      Serial.println(client.state());
      attempts++;
      delay(3000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("\n📨 [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.print(length);
  Serial.println(" bytes");
  
  if (length > 16000) {
    Serial.println("✗ Message too large!");
    return;
  }
  
  String message = "";
  message.reserve(length + 100);
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  DynamicJsonDocument doc(20480);
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("✗ JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }
  
  Serial.println("✓ JSON parsed");
  
  if (strcmp(topic, topic_gcode) == 0) {
    String command = doc["command"];
    
    if (command == "batch") {
      Serial.println("\n📦 BATCH G-CODE RECEIVED");
      
      batchStartLine = doc["start_line"] | 0;
      int numLines = doc["num_lines"] | 0;
      String gcodeData = doc["gcode_data"] | "";
      
      Serial.println("   Start: " + String(batchStartLine));
      Serial.println("   Lines: " + String(numLines));
      
      // ✅ FIXED: Clear buffer SETIAP batch (proses per batch, bukan akumulatif)
      clearGcodeBuffer();
      Serial.println("   ✓ Buffer cleared for new batch");
      
      int lineCount = 0;
      int startPos = 0;
      int endPos = 0;
      
      while (endPos != -1 && lineCount < numLines) {
        endPos = gcodeData.indexOf('\n', startPos);
        String line;
        
        if (endPos == -1) {
          line = gcodeData.substring(startPos);
        } else {
          line = gcodeData.substring(startPos, endPos);
        }
        
        line.trim();
        if (line.length() > 0) {
          gcodeBuffer.push_back(line);
          lineCount++;
        }
        
        startPos = endPos + 1;
      }
      
      totalLines = gcodeBuffer.size();
      batchReceiveTime = millis();
      
      Serial.println("✓ Parsed: " + String(lineCount) + " lines");
      Serial.println("✓ Total buffer: " + String(totalLines) + " lines");
      
      // ✅ FIXED: Set flag untuk auto-start batch ini
      batchReceived = true;
      autoStartBatch = true;
      Serial.println("\n✅ Batch ready - Will auto-start!");
      
      StaticJsonDocument<256> ackDoc;
      ackDoc["status"] = "batch_received";
      ackDoc["start_line"] = batchStartLine;
      ackDoc["received_lines"] = lineCount;
      ackDoc["total_buffered"] = totalLines;
      
      String ackMsg;
      serializeJson(ackDoc, ackMsg);
      client.publish(topic_status, ackMsg.c_str());
      
    } else if (command == "manual") {
      String gcode = doc["gcode"];
      processGcode(gcode);
      delay(100);
      sendPositionUpdate();
    }
    
  } else if (strcmp(topic, topic_control) == 0) {
    String command = doc["command"];
    
    if (command == "start") {
      if (!isRunning && batchReceived) {
        isRunning = true;
        isPaused = false;
        currentLine = 0;
        Serial.println("\n▶️  MANUAL START");
        sendStatus("started");
        enableMotors(true);
      }
    } else if (command == "stop") {
      isRunning = false;
      isPaused = false;
      currentLine = 0;
      autoStartBatch = false; // ✅ FIXED
      Serial.println("\n⏹️  STOPPED");
      sendStatus("stopped");
      enableMotors(false);
      setServoZ(SERVO_PEN_UP);
    } else if (command == "pause") {
      isPaused = !isPaused;
      sendStatus(isPaused ? "paused" : "running");
    } else if (command == "home") {
      homeAll();
      sendPositionUpdate();
    } else if (command == "clear_buffer") {
      clearGcodeBuffer();
      sendStatus("buffer_cleared");
    }
  }
}

void processBatchGcode() {
  if (currentLine >= totalLines) {
    Serial.println("\n✅ BATCH COMPLETED!");
    Serial.println("   Processed: " + String(totalLines) + " lines");
    
    // ✅ FIXED: Clear buffer DULU sebelum kirim status
    int processedLines = totalLines;
    clearGcodeBuffer();
    Serial.println("   ✓ Buffer cleared, ready for next batch");
    
    isRunning = false;
    batchReceived = false;
    
    Serial.println("   Flags after completion:");
    Serial.println("     isRunning: " + String(isRunning));
    Serial.println("     batchReceived: " + String(batchReceived));
    Serial.println("     autoStartBatch: " + String(autoStartBatch));
    Serial.println("     totalLines: " + String(totalLines));
    Serial.println();
    
    // ✅ FIXED: JANGAN disable motors dan angkat pen
    // Biarkan tetap aktif untuk batch berikutnya
    // enableMotors(false);  // DIHAPUS
    // setServoZ(SERVO_PEN_UP);  // DIHAPUS
    
    sendStatus("completed");
    
    return;
  }
  
  if (isPaused) return;
  
  String gcode = gcodeBuffer[currentLine];
  
  if (currentLine % PROGRESS_UPDATE_INTERVAL == 0 || currentLine == totalLines - 1) {
    sendProgressUpdate(currentLine + 1, totalLines);
  }
  
  processGcode(gcode);
  currentLine++;
  
  if (currentLine % 50 == 0) {
    client.loop();
    delay(5);
  }
}

void clearGcodeBuffer() {
  gcodeBuffer.clear();
  currentLine = 0;
  totalLines = 0;
  batchReceived = false;
  autoStartBatch = false; // ✅ FIXED
  Serial.println("✓ Buffer cleared");
}

void sendProgressUpdate(int line, int total) {
  StaticJsonDocument<256> doc;
  doc["line"] = line;
  doc["total"] = total;
  doc["progress"] = (line * 100) / total;
  doc["x"] = currentX;
  doc["y"] = currentY;
  doc["z"] = currentZ;
  
  String message;
  serializeJson(doc, message);
  client.publish(topic_progress, message.c_str());
}

void processGcode(String gcode) {
  gcode.trim();
  gcode.toUpperCase();
  
  if (gcode.length() == 0) return;
  
  int commentIndex = gcode.indexOf(';');
  if (commentIndex != -1) {
    gcode = gcode.substring(0, commentIndex);
    gcode.trim();
  }
  
  if (gcode.startsWith("G0") || gcode.startsWith("G1")) {
    float x = currentX, y = currentY, z = currentZ;
    int feedRate = 1000;
    bool hasX = false, hasY = false, hasZ = false;
    
    if (hasCoordinate(gcode, 'X')) { x = parseCoordinate(gcode, 'X'); hasX = true; }
    if (hasCoordinate(gcode, 'Y')) { y = parseCoordinate(gcode, 'Y'); hasY = true; }
    if (hasCoordinate(gcode, 'Z')) { z = parseCoordinate(gcode, 'Z'); hasZ = true; }
    if (hasCoordinate(gcode, 'F')) { feedRate = (int)parseCoordinate(gcode, 'F'); }
    
    if (hasZ) setServoZ(z);
    if (hasX || hasY) {
      if (absoluteMode) {
        moveToPosition(x, y, feedRate);
      } else {
        moveRelative(hasX ? x : 0, hasY ? y : 0, feedRate);
      }
    }
  } else if (gcode.startsWith("G90")) {
    absoluteMode = true;
  } else if (gcode.startsWith("G91")) {
    absoluteMode = false;
  } else if (gcode.startsWith("G92")) {
    if (hasCoordinate(gcode, 'X')) currentX = parseCoordinate(gcode, 'X');
    if (hasCoordinate(gcode, 'Y')) currentY = parseCoordinate(gcode, 'Y');
    if (hasCoordinate(gcode, 'Z')) currentZ = parseCoordinate(gcode, 'Z');
  } else if (gcode.startsWith("G28")) {
    homeAll();
  } else if (gcode.startsWith("M3") || gcode.startsWith("M4")) {
    float angle = SERVO_PEN_DOWN;
    if (hasCoordinate(gcode, 'S')) angle = parseCoordinate(gcode, 'S');
    setServoZ(angle, true);
  } else if (gcode.startsWith("M5")) {
    float angle = SERVO_PEN_UP;
    if (hasCoordinate(gcode, 'S')) angle = parseCoordinate(gcode, 'S');
    setServoZ(angle, true);
  } else if (gcode.startsWith("M17")) {
    enableMotors(true);
  } else if (gcode.startsWith("M18") || gcode.startsWith("M84")) {
    enableMotors(false);
  } else if (gcode.startsWith("M112")) {
    emergencyStop();
  }
}

void moveToPosition(float targetX, float targetY, int feedRate) {
  targetX = constrain(targetX, 0, MAX_X);
  targetY = constrain(targetY, 0, MAX_Y);
  
  float deltaX = targetX - currentX;
  float deltaY = targetY - currentY;
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
  
  if (distance < 0.01) return;
  if (!motorsEnabled) enableMotors(true);
  
  if (feedRate > MAX_SPEED) feedRate = MAX_SPEED;
  if (feedRate < MIN_SPEED) feedRate = MIN_SPEED;
  
  // CoreXY Kinematics
  float currentA = currentX + currentY;
  float currentB = currentX - currentY;
  float targetA = targetX + targetY;
  float targetB = targetX - targetY;
  
  float deltaA = targetA - currentA;
  float deltaB = targetB - currentB;
  
  int stepsA = abs(deltaA * STEPS_PER_MM);
  int stepsB = abs(deltaB * STEPS_PER_MM);
  
  if (stepsA == 0 && stepsB == 0) return;
  
  bool dirA = deltaA > 0;
  if (invertX) dirA = !dirA;
  digitalWrite(DIR_X, dirA ? LOW : HIGH);
  
  bool dirB = deltaB > 0;
  if (invertY) dirB = !dirB;
  digitalWrite(DIR_Y, dirB ? LOW : HIGH);
  
  long delayMicros = (60L * 1000000L) / ((long)feedRate * STEPS_PER_MM);
  if (delayMicros < 200) delayMicros = 200;
  if (delayMicros > 10000) delayMicros = 10000;
  
  int maxSteps = max(stepsA, stepsB);
  long errorA = maxSteps / 2;
  long errorB = maxSteps / 2;
  
  for (int i = 0; i < maxSteps; i++) {
    if (isRunning && isPaused) break;
    
    bool stepA = false, stepB = false;
    
    errorA += stepsA;
    if (errorA >= maxSteps) {
      errorA -= maxSteps;
      stepA = true;
    }
    
    errorB += stepsB;
    if (errorB >= maxSteps) {
      errorB -= maxSteps;
      stepB = true;
    }
    
    if (stepA) digitalWrite(STEP_X, HIGH);
    if (stepB) digitalWrite(STEP_Y, HIGH);
    delayMicroseconds(delayMicros / 2);
    if (stepA) digitalWrite(STEP_X, LOW);
    if (stepB) digitalWrite(STEP_Y, LOW);
    delayMicroseconds(delayMicros / 2);
  }
  
  currentX = targetX;
  currentY = targetY;
}

void moveRelative(float dx, float dy, int feedRate) {
  moveToPosition(currentX + dx, currentY + dy, feedRate);
}

void setServoZ(float zPos, bool fromGcode) {
  int servoAngle;
  
  if (fromGcode) {
    servoAngle = (int)zPos;
  } else {
    servoAngle = (zPos >= SERVO_PEN_UP) ? SERVO_PEN_UP : SERVO_PEN_DOWN;
  }
  
  if (invertZ) servoAngle = 180 - servoAngle;
  
  servoZ.write(servoAngle);
  currentZ = zPos;
  delay(300);
}

void homeAll() {
  setServoZ(SERVO_PEN_UP);
  delay(500);
  moveToPosition(0, 0, 1000);
  currentX = 0;
  currentY = 0;
  sendStatus("homed");
}

void enableMotors(bool enable) {
  if (enable) {
    digitalWrite(ENABLE_X, LOW);
    digitalWrite(ENABLE_Y, LOW);
    motorsEnabled = true;
  } else {
    digitalWrite(ENABLE_X, HIGH);
    digitalWrite(ENABLE_Y, HIGH);
    motorsEnabled = false;
  }
  sendPositionUpdate();
}

void emergencyStop() {
  Serial.println("🛑 EMERGENCY STOP!");
  enableMotors(false);
  setServoZ(SERVO_PEN_UP);
  isRunning = false;
  isPaused = false;
  sendStatus("emergency_stop");
}

void sendStatus(String status) {
  StaticJsonDocument<512> doc;
  doc["status"] = status;
  doc["line"] = currentLine;
  doc["total"] = totalLines;
  doc["x"] = currentX;
  doc["y"] = currentY;
  doc["z"] = currentZ;
  doc["motors"] = motorsEnabled;
  doc["running"] = isRunning;
  doc["paused"] = isPaused;
  doc["buffer_lines"] = gcodeBuffer.size();
  
  String message;
  serializeJson(doc, message);
  client.publish(topic_status, message.c_str());
}

void sendPositionUpdate() {
  StaticJsonDocument<512> doc;
  doc["x"] = currentX;
  doc["y"] = currentY;
  doc["z"] = currentZ;
  doc["motors"] = motorsEnabled;
  doc["running"] = isRunning;
  doc["paused"] = isPaused;
  doc["absolute"] = absoluteMode;
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["buffer_lines"] = gcodeBuffer.size();
  doc["current_line"] = currentLine;
  
  String message;
  serializeJson(doc, message);
  client.publish(topic_status, message.c_str());
}

float parseCoordinate(String gcode, char axis) {
  int index = gcode.indexOf(axis);
  if (index == -1) return 0;
  
  String valueStr = "";
  for (int i = index + 1; i < gcode.length(); i++) {
    char c = gcode.charAt(i);
    if (c == ' ' || c == 'X' || c == 'Y' || c == 'Z' || c == 'F' || c == 'S') break;
    valueStr += c;
  }
  
  return valueStr.toFloat();
}

bool hasCoordinate(String gcode, char axis) {
  return gcode.indexOf(axis) != -1;
}
