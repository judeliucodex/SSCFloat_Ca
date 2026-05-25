#include <Wire.h>
#include <MS5837.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <ArduinoOTA.h>
#include <Update.h>

// ==========================================
// --- I2C PINS & SENSOR CONFIG ---
// ==========================================
#define CUSTOM_SDA_PIN 21
#define CUSTOM_SCL_PIN 22

MS5837 sensor;
bool sensorActive = false;

// ==========================================
// --- WIFI CREDENTIALS (AP MODE) ---
// ==========================================
const char* ssid     = "SSCFLOAT-RN";     
const char* password = "12345678"; 
WebServer server(80);

// ==========================================
// --- TELEMETRY & DATA VARIABLES ---
// ==========================================
#define MAX_PACKETS 600   
#define SMA_SIZE 5        

char dataPackets[MAX_PACKETS][64]; 
int sensorIdx = 0;
int iterationCount = 0;

float currentRelativeDepth = 0.0;
float surfaceDepthOffset = 0.0;
float depthWindow[SMA_SIZE] = {0};
int windowIdx = 0;

// ==========================================
// --- WIRELESS SERIAL LIVE LOGGER ---
// ==========================================
SemaphoreHandle_t logLock = NULL;
String liveLogBuffer = "";

// ==========================================
// --- MOTOR & HARDWARE PINS ---
// ==========================================
const int dirPin  = 4;
const int stepPin = 16;
int motorSpeed = 2000; 

#define BUTTON_PIN_1 19    // Bottom / Sinking limit
#define BUTTON_PIN_2 18    // Top / Floating limit

// ==========================================
// --- POSITION TRACKING ---
// ==========================================
long currentStepperPos = 0;
volatile bool isHomed = false;

// ==========================================
// --- STATE MACHINE ---
// ==========================================
enum SystemMode {
  MODE_IDLE,
  MODE_HOMING,
  MODE_MANUAL_PULSE,
  MODE_FEEDFORWARD_SEQUENCE,
  MODE_FULL_BUMP_SEQUENCE,
  MODE_TARGET_STEP_SEQUENCE
};
volatile SystemMode currentMode = MODE_IDLE;

// ==========================================
// --- STATUS & MANUAL CONTROLS ---
// ==========================================
volatile long holdSecondsRemaining = 0;
String currentSubStatus = "IDLE";
bool manualDir = true;
uint32_t manualDurationMs = 0;
uint32_t manualStartTime = 0;
long feedforwardTargetSteps = 0;
long targetStepPos = 0; 

// ==========================================
// --- CONFIGURATION VARIABLES ---
// ==========================================
uint32_t floatWaitTimeMs = 30000;
uint32_t sensorReadIntervalMs = 50;
uint32_t sensorLogIntervalMs = 100;
float depthToleranceCm = 3.0; // Kept per user request

// ==========================================
// --- MUTEXES & TASK HANDLES ---
// ==========================================
SemaphoreHandle_t dataLock = NULL;
SemaphoreHandle_t modeLock = NULL;
SemaphoreHandle_t configLock = NULL;

TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t webTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t otaTaskHandle = NULL;


// ==========================================
// --- THREAD-SAFE WIRELESS LOGGING FUNCTION ---
// ==========================================
void safeLog(String msg) {
  Serial.println(msg);
  if (logLock != NULL && xSemaphoreTake(logLock, portMAX_DELAY)) {
    liveLogBuffer += "[" + String(millis() / 1000) + "s] " + msg + "\n";
    if (liveLogBuffer.length() > 3000) {
      liveLogBuffer = liveLogBuffer.substring(liveLogBuffer.length() - 1500); 
    }
    xSemaphoreGive(logLock);
  }
}


// ==========================================
// --- STEPPER HELPER FUNCTION ---
// ==========================================
bool singleStep(bool dirHigh) {
  if (dirHigh) {
    if (digitalRead(BUTTON_PIN_1) == LOW) return false;
    digitalWrite(dirPin, HIGH);
  } else {
    if (digitalRead(BUTTON_PIN_2) == LOW) return false;
    digitalWrite(dirPin, LOW);
  }

  digitalWrite(stepPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(stepPin, LOW);

  int delayMs = motorSpeed / 1000;
  if (delayMs > 0) {
    vTaskDelay(pdMS_TO_TICKS(delayMs));
  } else {
    delayMicroseconds(motorSpeed);
  }

  if (dirHigh) currentStepperPos++;
  else currentStepperPos--;
  
  return true;
}


// ==========================================
// --- SENSOR TASK (CORE 0) ---
// ==========================================
void sensorTask(void *parameter) {
  static uint32_t lastLogTime = 0;

  while(1) {
    if (sensorActive) {
      sensor.read();
      if (xSemaphoreTake(dataLock, portMAX_DELAY)) {
        float rawRelativeDepth = -(sensor.depth() - surfaceDepthOffset);

        // Simple Moving Average (SMA) Filter
        depthWindow[windowIdx] = rawRelativeDepth;
        windowIdx = (windowIdx + 1) % SMA_SIZE;
        float sum = 0;
        for(int i = 0; i < SMA_SIZE; i++) {
          sum += depthWindow[i];
        }
        currentRelativeDepth = sum / SMA_SIZE;

        uint32_t logInt = 100;
        if (xSemaphoreTake(configLock, portMAX_DELAY)) {
          logInt = sensorLogIntervalMs;
          xSemaphoreGive(configLock);
        }

        if (millis() - lastLogTime >= logInt) {
          snprintf(dataPackets[sensorIdx], 64, "RN10 %lus %.1fhPa %.2fm Pos:%ld", 
                   (millis() / 1000), 
                   sensor.pressure(), 
                   currentRelativeDepth, 
                   currentStepperPos);
                   
          sensorIdx = (sensorIdx + 1) % MAX_PACKETS;
          if(iterationCount < MAX_PACKETS) iterationCount++;
          lastLogTime = millis();
        }
        xSemaphoreGive(dataLock);
      }
    }

    uint32_t delayMs = 50;
    if (xSemaphoreTake(configLock, portMAX_DELAY)) {
      delayMs = sensorReadIntervalMs;
      xSemaphoreGive(configLock);
    }
    vTaskDelay(pdMS_TO_TICKS(delayMs));
  }
}


// ==========================================
// --- WEB SERVER TASK & ENDPOINTS ---
// ==========================================
void webTask(void *parameter) {
  while(1) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void handleData() {
  if (xSemaphoreTake(dataLock, portMAX_DELAY)) {
    String data = "Status: " + currentSubStatus + " | Hold Rem: " + String(holdSecondsRemaining) + "s\n";
    data += "Current Depth: " + String(currentRelativeDepth * 100.0) + " cm\n";
    
    int total = iterationCount < MAX_PACKETS ? iterationCount : MAX_PACKETS;
    for (int i = 0; i < total; i++) {
      int idx = (sensorIdx - 1 - i + MAX_PACKETS) % MAX_PACKETS;
      data += String(dataPackets[idx]) + "\n";
    }
    xSemaphoreGive(dataLock);
    server.send(200, "text/plain", data);
  }
}

void handleLivePage() {
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Float Live Serial Terminal</title>";
  html += "<style>body{font-family:'Courier New',monospace; background:#121212; color:#00ff66; padding:20px; margin:0;} ";
  html += "#terminal{background:#000000; border:2px solid #333; border-radius:8px; padding:15px; height:82vh; overflow-y:scroll; white-space:pre-wrap; box-shadow: inset 0 0 10px #00ff6633;} ";
  html += "h2{margin-top:0; color:#ffffff; font-family:Arial,sans-serif;}</style></head>";
  html += "<body><h2>MATE ROV Wireless Serial Terminal</h2>";
  html += "<div id='terminal'>Awaiting data feed connection...</div>";
  html += "<script>";
  html += "function updateOutput(){";
  html += "  fetch('/livedata').then(response=>response.text()).then(text=>{";
  html += "    let term=document.getElementById('terminal');";
  html += "    term.innerText=text;";
  html += "    term.scrollTop=term.scrollHeight;"; 
  html += "  });";
  html += "}";
  html += "setInterval(updateOutput, 500);"; 
  html += "updateOutput();";
  html += "</script></body></html>";
  server.send(200, "text/html", html);
}

void handleLiveData() {
  if (xSemaphoreTake(logLock, portMAX_DELAY)) {
    server.send(200, "text/plain", liveLogBuffer);
    xSemaphoreGive(logLock);
  } else {
    server.send(500, "text/plain", "Log system busy");
  }
}

void handleControl() {
  if(server.hasArg("action")) {
    String action = server.arg("action");
    if (xSemaphoreTake(modeLock, portMAX_DELAY)) {
      if (action == "stop") {
        currentMode = MODE_IDLE;
        safeLog("[E-STOP] Emergency Stop triggered by user!");
      } 
      else if (action == "home") {
        currentMode = MODE_HOMING;
        isHomed = false;
        safeLog("[SYSTEM] Initiating syringe homing procedure...");
      } 
      else if (action == "manual") {
        manualDir = (server.arg("dir") == "down");
        manualDurationMs = server.arg("time").toInt() * 1000;
        manualStartTime = millis();
        currentMode = MODE_MANUAL_PULSE;
        safeLog("[MANUAL] Pulsing motor. Direction: " + String(manualDir ? "DOWN" : "UP"));
      } 
      else if (action == "feedforward" && isHomed) {
        feedforwardTargetSteps = server.arg("steps").toInt();
        currentMode = MODE_FEEDFORWARD_SEQUENCE;
        safeLog("[SEQUENCE] Starting Feedforward sequence. Relative Steps: " + String(feedforwardTargetSteps));
      } 
      else if (action == "targetstep" && isHomed) {
        targetStepPos = server.arg("steps").toInt();
        currentMode = MODE_TARGET_STEP_SEQUENCE;
        safeLog("[SEQUENCE] Starting Target Step sequence. Absolute Target: " + String(targetStepPos));
      }
      else if (action == "fullbump" && isHomed) {
        currentMode = MODE_FULL_BUMP_SEQUENCE;
        safeLog("[SEQUENCE] Starting Full Bump sequence.");
      } 
      xSemaphoreGive(modeLock);
    }
    server.sendHeader("Location", "/control");
    server.send(303);
    return;
  }

  SystemMode modeCopy;
  if (xSemaphoreTake(modeLock, portMAX_DELAY)) {
    modeCopy = currentMode;
    xSemaphoreGive(modeLock);
  }

  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial; text-align:center;} button { padding: 15px; margin: 10px auto; font-size: 18px; width: 90%; max-width: 400px; display: block; border-radius: 5px; border: none; cursor: pointer; font-weight: bold;} ";
  html += "input, select { padding:12px; margin:5px auto; font-size:16px; width:90%; max-width:400px; display: block; border-radius: 5px; border: 1px solid #ccc; box-sizing: border-box; }";
  html += ".stop { background-color: #ff4c4c; color: white; } .manual { background-color: #333; color: white; } .config { background-color: #4CAF50; color: white; }";
  html += "</style></head><body><h2>MATE ROV Float Controller</h2>";

  if (modeCopy == MODE_FEEDFORWARD_SEQUENCE || modeCopy == MODE_FULL_BUMP_SEQUENCE || modeCopy == MODE_TARGET_STEP_SEQUENCE) {
    html += "<div style='padding:15px; background:#e3f2fd; border:2px solid #2196f3; border-radius:10px; margin:10px auto; width:90%; max-width:400px;'>";
    html += "Phase: <strong style='color:#9c27b0;'>" + currentSubStatus + "</strong><br>";
    if (currentSubStatus == "HOLDING") {
      html += "<span style='font-size:2em; color:#2e7d32;'><strong>" + String(holdSecondsRemaining) + "s</strong> Left</span>";
    }
    html += "</div>";
  } else {
    html += "<p>Status: <strong>" + String(modeCopy == MODE_IDLE ? "IDLE" : (modeCopy == MODE_HOMING ? "HOMING" : "MANUAL")) + "</strong></p>";
  }

  html += "<button class='stop' onclick=\"location.href='/control?action=stop'\">EMERGENCY STOP</button>";
  html += "<button class='config' style='background-color:#008CBA;' onclick=\"location.href='/control?action=home'\">RE-HOME SYRINGES</button>";

  html += "<hr><h3>Manual Motor Recovery</h3><form action='/control' method='GET'><input type='hidden' name='action' value='manual'>";
  html += "<select name='dir'><option value='up'>UP (Float)</option><option value='down'>DOWN (Sink)</option></select>";
  html += "<input type='number' name='time' placeholder='Seconds' required><button type='submit' class='manual'>GO</button></form>";

  html += "<hr><h3>Feedforward (Relative Steps)</h3><form action='/control' method='GET'><input type='hidden' name='action' value='feedforward'>";
  html += "<input type='number' name='steps' placeholder='Steps Down (e.g. 8000)' required min='1'>";
  if (isHomed) html += "<button type='submit' class='manual' style='background-color:#ff9800;'>Run Feedforward</button></form>";
  else html += "<button type='button' class='manual' style='background:#ccc' disabled>Home First</button></form>";

  html += "<hr><h3>Run To Target Step (Absolute)</h3><form action='/control' method='GET'><input type='hidden' name='action' value='targetstep'>";
  html += "<input type='number' name='steps' placeholder='Target Absolute Step (e.g. 15000)' required min='0'>";
  if (isHomed) html += "<button type='submit' class='manual' style='background-color:#673ab7;'>Run to Step</button></form>";
  else html += "<button type='button' class='manual' style='background:#ccc' disabled>Home First</button></form>";

  html += "<hr><h3>Full Sequence (Bump & Hold)</h3><form action='/control' method='GET'><input type='hidden' name='action' value='fullbump'>";
  if (isHomed) html += "<button type='submit' class='manual' style='background-color:#009688;'>Run Full Sequence</button></form>";
  else html += "<button type='button' class='manual' style='background:#ccc' disabled>Home First</button></form>";

  html += "<hr><button class='config' onclick=\"location.href='/config'\">Settings</button>";
  html += "<br><br><button class='manual' style='background-color:#00ff66; color:black;' onclick=\"window.open('/live')\">OPEN LIVE WIRELESS TERMINAL</button>";
  html += "</body></html>"; 
  
  server.send(200, "text/html", html);
}

void handleConfig() {
  if (server.method() == HTTP_POST || server.args() > 0) {
    if (xSemaphoreTake(configLock, portMAX_DELAY)) {
      if (server.hasArg("waitTime")) floatWaitTimeMs = server.arg("waitTime").toInt();
      if (server.hasArg("tolerance")) depthToleranceCm = server.arg("tolerance").toFloat();
      if (server.hasArg("readInterval")) sensorReadIntervalMs = server.arg("readInterval").toInt();
      if (server.hasArg("logInterval")) sensorLogIntervalMs = server.arg("logInterval").toInt();
      if (server.hasArg("motorDelay")) motorSpeed = server.arg("motorDelay").toInt(); 
      xSemaphoreGive(configLock);
    }
    server.sendHeader("Location", "/config");
    server.send(303);
    return;
  }

  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial; padding: 20px;} input { padding:10px; margin:5px 0 15px 0; width:100%; max-width:300px; display:block; } button { padding: 12px 20px; font-size: 16px; background-color: #4CAF50; color: white; border: none; cursor: pointer; }</style></head>";
  html += "<body><h2>System Settings</h2><form action='/config' method='POST'>";

  html += "<label>Motor Step Delay (us/ms): <small>(Global Speed)</small></label>";
  html += "<input type='number' name='motorDelay' value='"+String(motorSpeed)+"'>";

  html += "<label>Hold Time (ms):</label><input type='number' name='waitTime' value='"+String(floatWaitTimeMs)+"'>";
  
  html += "<label>Depth Tolerance (cm):</label><input type='number' step='0.1' name='tolerance' value='"+String(depthToleranceCm)+"'>";

  html += "<hr>";

  html += "<label>Sensor Read Interval (ms):</label>";
  html += "<input type='number' name='readInterval' value='"+String(sensorReadIntervalMs)+"'>";

  html += "<label>Data Log Interval (ms): <small>(Telemetry save rate)</small></label>";
  html += "<input type='number' name='logInterval' value='"+String(sensorLogIntervalMs)+"'>";

  html += "<button type='submit'>Save Settings</button></form></body></html>";
  server.send(200, "text/html", html);
}


// ==========================================
// --- MOTOR BEHAVIORS & SEQUENCES ---
// ==========================================
void surfaceEmergency() {
  safeLog("[SAFETY] Surfacing active. Driving upward to limit switch...");
  while(digitalRead(BUTTON_PIN_2) == HIGH && currentMode != MODE_HOMING && currentMode != MODE_IDLE) {
    singleStep(false); 
    vTaskDelay(1);
  }
  if (digitalRead(BUTTON_PIN_2) == LOW) {
    safeLog("[SAFETY] Upper limit switch triggered successfully.");
  }
}

// Helper function to cleanly handle the holding timer and resurfacing for all sequences
void executeHoldAndSurface(SystemMode expectedMode, String logPrefix) {
  if (currentMode == expectedMode) {
    currentSubStatus = "HOLDING";
    safeLog("[" + logPrefix + " STATUS] Target reached. Holding.");
    uint32_t holdTimeMs = 30000;
    
    if (xSemaphoreTake(configLock, portMAX_DELAY)) {
      holdTimeMs = floatWaitTimeMs;
      xSemaphoreGive(configLock);
    }

    uint32_t startHold = millis();
    while (millis() - startHold < holdTimeMs && currentMode == expectedMode) {
      holdSecondsRemaining = (holdTimeMs / 1000) - ((millis() - startHold) / 1000);
      vTaskDelay(100);
    }
  }

  if (currentMode == expectedMode) {
    currentSubStatus = "SURFACING";
    surfaceEmergency();
  }
  
  currentSubStatus = "IDLE";
  if (xSemaphoreTake(modeLock, portMAX_DELAY)) {
    currentMode = MODE_IDLE;
    xSemaphoreGive(modeLock);
  }
  safeLog("[" + logPrefix + " STATUS] Sequence complete. System IDLE.");
}


void runFeedforwardSequence() {
  currentSubStatus = "SINKING (FF)";
  safeLog("[FF STATUS] Phase changed to: SINKING");
  long stepsMoved = 0;

  while (stepsMoved < feedforwardTargetSteps && currentMode == MODE_FEEDFORWARD_SEQUENCE) {
    if (digitalRead(BUTTON_PIN_1) == LOW) {
      safeLog("[WARNING] Bottom limit hit during Feedforward intake.");
      break; 
    }
    if (singleStep(true)) {
      stepsMoved++;
    }
  }
  
  executeHoldAndSurface(MODE_FEEDFORWARD_SEQUENCE, "FF");
}

void runTargetStepSequence() {
  currentSubStatus = "MOVING";
  safeLog("[STEP STATUS] Phase changed to: MOVING to step " + String(targetStepPos));

  while (currentMode == MODE_TARGET_STEP_SEQUENCE && currentStepperPos != targetStepPos) {
    bool dirDown = (targetStepPos > currentStepperPos); 
    
    if (dirDown && digitalRead(BUTTON_PIN_1) == LOW) {
      safeLog("[WARNING] Bottom limit hit before reaching target step!");
      break;
    }
    if (!dirDown && digitalRead(BUTTON_PIN_2) == LOW) {
      safeLog("[WARNING] Top limit hit before reaching target step!");
      break;
    }
    singleStep(dirDown);
  }

  executeHoldAndSurface(MODE_TARGET_STEP_SEQUENCE, "STEP");
}

void runFullBumpSequence() {
  currentSubStatus = "SINKING (BUMP)";
  safeLog("[BUMP STATUS] Phase changed to: SINKING");

  while (currentMode == MODE_FULL_BUMP_SEQUENCE) {
    if (digitalRead(BUTTON_PIN_1) == LOW) {
      safeLog("[BUMP STATUS] Limit switch reached. Sinking complete.");
      break;
    }
    singleStep(true);
  }

  executeHoldAndSurface(MODE_FULL_BUMP_SEQUENCE, "BUMP");
}


// ==========================================
// --- MAIN MOTOR TASK (CORE 1) ---
// ==========================================
void motorTask(void *parameter) {
  while(1) {
    SystemMode m;
    if (xSemaphoreTake(modeLock, portMAX_DELAY)) {
      m = currentMode;
      xSemaphoreGive(modeLock);
    }

    if (m == MODE_HOMING) {
      while(digitalRead(BUTTON_PIN_2) == HIGH && currentMode == MODE_HOMING) {
        singleStep(false);
        vTaskDelay(1);
      }
      if (currentMode == MODE_HOMING) {
        currentStepperPos = 0;
        isHomed = true;
        safeLog("[SYSTEM] Home reference set to 0 steps.");
      }
      
      if (xSemaphoreTake(modeLock, portMAX_DELAY)) {
        currentMode = MODE_IDLE;
        xSemaphoreGive(modeLock);
      }
    }
    else if (m == MODE_MANUAL_PULSE) {
      while(millis() - manualStartTime < manualDurationMs) {
        if (!singleStep(manualDir) || currentMode != MODE_MANUAL_PULSE) break;
        vTaskDelay(1);
      }
      
      if (xSemaphoreTake(modeLock, portMAX_DELAY)) {
        currentMode = MODE_IDLE;
        xSemaphoreGive(modeLock);
      }
    }
    else if (m == MODE_FEEDFORWARD_SEQUENCE) {
      runFeedforwardSequence();
    }
    else if (m == MODE_TARGET_STEP_SEQUENCE) { 
      runTargetStepSequence();
    }
    else if (m == MODE_FULL_BUMP_SEQUENCE) {
      runFullBumpSequence();
    }
    else {
      vTaskDelay(100);
    }
  }
}

// ==========================================
// --- DEDICATED OTA & DISCOVERY TASK (CORE 0) ---
// ==========================================
void otaTask(void *parameter) {
  while(1) {
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}


// ==========================================
// --- SETUP & LOOP ---
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);

  logLock = xSemaphoreCreateMutex();
  safeLog("[BOOT] System initialization started...");

  unsigned long start = millis();
  while (millis() - start < 5000) {
    Wire.beginTransmission(0x76);
    if (Wire.endTransmission() == 0 && sensor.init()) {
      sensorActive = true;
      sensor.setFluidDensity(997); 
      sensor.read();
      surfaceDepthOffset = sensor.depth();
      safeLog("[BOOT] MS5837 pressure sensor baseline loaded successfully.");
      break;
    }
    delay(250);
  }

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  delay(500); 

  safeLog("[NET] Access Point started successfully!");
  safeLog("[NET] Network Name: " + String(ssid));
  safeLog("[NET] AP IP Address: " + WiFi.softAPIP().toString());

  if (MDNS.begin("sscfloat")) {
    safeLog("[NET] mDNS domain initialized: http://sscfloat.local");
    MDNS.addService("arduino", "tcp", 3232);
    MDNS.addService("http", "tcp", 80);
  }

  ArduinoOTA.begin();
  
  server.on("/data", handleData);
  server.on("/control", handleControl);
  server.on("/config", handleConfig);
  server.on("/live", handleLivePage);       
  server.on("/livedata", handleLiveData);   
  server.begin();

  dataLock = xSemaphoreCreateMutex();
  modeLock = xSemaphoreCreateMutex();
  configLock = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(sensorTask, "Sens", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(webTask,   "Web",  4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(otaTask,   "OTA",  4096, NULL, 3, &otaTaskHandle, 0); 
  xTaskCreatePinnedToCore(motorTask, "Mot",  4096, NULL, 3, NULL, 1);           
  
  safeLog("[BOOT] FreeRTOS task schedules active. Float engine ready.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}