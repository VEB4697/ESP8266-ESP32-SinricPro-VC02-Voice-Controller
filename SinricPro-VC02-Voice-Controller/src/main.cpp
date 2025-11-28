/*
 * ESP8266/ESP32 SinricPro Multi-Switch with VC02 Voice Controller Integration
 * Features:
 * - WiFi connection stability with delayed relay response
 * - UART communication with VC02 voice controller
 * - Manual flip switch control
 * - Cloud sync via SinricPro
 * 
 * Hardware UART Commands from VC02:
 * - AA00 = LED OFF (Device 1)
 * - AA11 = LED ON  (Device 1)
 * - BB00 = FAN OFF (Device 2)
 * - BB11 = FAN ON  (Device 2)
 */

#ifdef ENABLE_DEBUG
  #define DEBUG_ESP_PORT Serial
  #define NODEBUG_WEBSOCKETS
  #define NDEBUG
#endif 

#include <Arduino.h>
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <SoftwareSerial.h>
#elif defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
  #include <WiFi.h>
  #include <HardwareSerial.h>
#endif

#include "SinricPro.h"
#include "SinricProSwitch.h"
#include <map>

// ====== WiFi Configuration ======
#define WIFI_SSID         "vaibhav"
#define WIFI_PASS         "VEB@4697"
#define APP_KEY           "f1158241-af33-4bdf-b8bf-067c42491146"
#define APP_SECRET        "d49edcd2-aee5-43e1-abc5-6b8d6f34644c-96ca2347-8633-43a8-bdc3-c477de4d9db1"

// ====== UART Configuration for VC02 ======
#if defined(ESP8266)
  // ESP8266: Using SoftwareSerial on D6(RX), D7(TX)
  #define VC02_RX_PIN 1  // Connect to VC02 TX
  #define VC02_TX_PIN 3  // Connect to VC02 RX
  SoftwareSerial VC02Serial(VC02_RX_PIN, VC02_TX_PIN);
#elif defined(ESP32)
  // ESP32: Using Hardware Serial2 on GPIO16(RX), GPIO17(TX)
  #define VC02_RX_PIN 16  // Connect to VC02 TX
  #define VC02_TX_PIN 17  // Connect to VC02 RX
  HardwareSerial VC02Serial(2);
#endif

#define VC02_BAUD_RATE 9600

// ====== Device Configuration ======
#define TACTILE_BUTTON 1  // Comment out if using toggle switches
#define BAUD_RATE   115200
#define DEBOUNCE_TIME 250

// WiFi Stability Settings
#define WIFI_CHECK_INTERVAL 5000    // Check WiFi every 5 seconds
#define MIN_WIFI_STRENGTH -70       // Minimum WiFi signal strength (dBSSI)
#define RELAY_RESPONSE_DELAY 300    // Delay before relay responds (ms)

typedef struct {
  int relayPIN;
  int flipSwitchPIN;
  bool activeLow;
  String deviceName;  // "LED" or "FAN"
} deviceConfig_t;

// Device Configuration Map
// Device 1 (LED): GPIO5 relay, GPIO14 switch
// Device 2 (FAN): GPIO4 relay, GPIO12 switch
std::map<String, deviceConfig_t> devices = {
    // {deviceId, {relayPIN, flipSwitchPIN, activeLow, deviceName}}
    {"688c44e8ddd2551252bb6f0f", {5, 14, false, "LED"}},
    {"688c44acedeca866fe97c5f1", {4, 12, false, "FAN"}},
};

typedef struct {
  String deviceId;
  bool lastFlipSwitchState;
  unsigned long lastFlipSwitchChange;
  bool activeLow;
} flipSwitchConfig_t;

std::map<int, flipSwitchConfig_t> flipSwitches;

// WiFi Status Variables
unsigned long lastWiFiCheck = 0;
bool wifiConnected = false;
bool sinricConnected = false;

// UART Command Buffer
String uartBuffer = "";

// ====== Function Prototypes ======
void setupRelays();
void setupFlipSwitches();
void setupWiFi();
void setupSinricPro();
void setupVC02Serial();
void checkWiFiConnection();
void handleFlipSwitches();
void handleVC02Commands();
void processVC02Command(String command);
void sendStateToVC02(String device, bool state);
bool onPowerState(String deviceId, bool &state);
String getDeviceIdByName(String deviceName);

// ====== Setup Functions ======
void setupRelays() { 
  for (auto &device : devices) {
    int relayPIN = device.second.relayPIN;
    pinMode(relayPIN, OUTPUT);
    digitalWrite(relayPIN, LOW); // Initialize relays to OFF
  }
  Serial.println("[Setup] Relays initialized");
}

void setupFlipSwitches() {
  for (auto &device : devices) {
    flipSwitchConfig_t flipSwitchConfig;
    flipSwitchConfig.deviceId = device.first;
    flipSwitchConfig.lastFlipSwitchChange = 0;
    flipSwitchConfig.lastFlipSwitchState = false;
    
    int flipSwitchPIN = device.second.flipSwitchPIN;
    bool activeLow = device.second.activeLow;
    flipSwitchConfig.activeLow = activeLow;
    flipSwitches[flipSwitchPIN] = flipSwitchConfig;
    
    if(activeLow) {
      pinMode(flipSwitchPIN, INPUT_PULLUP);
    } else {
      pinMode(flipSwitchPIN, INPUT);
    }
  }
  Serial.println("[Setup] Flip switches initialized");
}

void setupWiFi() {
  Serial.printf("\r\n[WiFi] Connecting to %s", WIFI_SSID);

  #if defined(ESP8266)
    WiFi.setSleepMode(WIFI_NONE_SLEEP); 
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
  #elif defined(ESP32)
    WiFi.setSleep(false); 
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
  #endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    Serial.print(".");
    delay(500);
    attempts++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.printf("\r\n[WiFi] Connected! IP: %s\r\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] Signal Strength: %d dBm\r\n", WiFi.RSSI());
  } else {
    wifiConnected = false;
    Serial.println("\r\n[WiFi] Connection failed! Operating in offline mode.");
  }
}

void setupSinricPro() {
  if(!wifiConnected) {
    Serial.println("[SinricPro] Skipping setup - No WiFi connection");
    return;
  }

  for (auto &device : devices) {
    const char *deviceId = device.first.c_str();
    SinricProSwitch &mySwitch = SinricPro[deviceId];
    mySwitch.onPowerState(onPowerState);
  }

  SinricPro.onConnected([]() {
    sinricConnected = true;
    Serial.println("[SinricPro] Connected to cloud");
  });

  SinricPro.onDisconnected([]() {
    sinricConnected = false;
    Serial.println("[SinricPro] Disconnected from cloud");
  });

  SinricPro.begin(APP_KEY, APP_SECRET);
  Serial.println("[SinricPro] Setup complete");
}

void setupVC02Serial() {
  #if defined(ESP8266)
    VC02Serial.begin(VC02_BAUD_RATE);
  #elif defined(ESP32)
    VC02Serial.begin(VC02_BAUD_RATE, SERIAL_8N1, VC02_RX_PIN, VC02_TX_PIN);
  #endif
  
  Serial.println("[VC02] UART initialized");
  Serial.printf("[VC02] RX Pin: %d, TX Pin: %d, Baud: %d\r\n", 
                VC02_RX_PIN, VC02_TX_PIN, VC02_BAUD_RATE);
}

// ====== WiFi Management ======
void checkWiFiConnection() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = currentMillis;
    
    if (WiFi.status() != WL_CONNECTED) {
      if(wifiConnected) {
        Serial.println("[WiFi] Connection lost! Reconnecting...");
        wifiConnected = false;
        sinricConnected = false;
      }
      
      WiFi.reconnect();
      delay(100);
      
    } else {
      // Check signal strength
      int32_t rssi = WiFi.RSSI();
      
      if(!wifiConnected) {
        wifiConnected = true;
        Serial.printf("[WiFi] Reconnected! IP: %s\r\n", WiFi.localIP().toString().c_str());
        
        // Reinitialize SinricPro after reconnection
        if(!sinricConnected) {
          setupSinricPro();
        }
      }
      
      if(rssi < MIN_WIFI_STRENGTH) {
        Serial.printf("[WiFi] Weak signal: %d dBm\r\n", rssi);
      }
    }
  }
}

// ====== Device Control Functions ======
bool onPowerState(String deviceId, bool &state) {
  // Add delay to ensure WiFi stability before responding
  if(wifiConnected) {
    delay(RELAY_RESPONSE_DELAY);
  }
  
  Serial.printf("[SinricPro] Device %s: %s\r\n", 
                deviceId.c_str(), state ? "ON" : "OFF");
  
  int relayPIN = devices[deviceId].relayPIN;
  digitalWrite(relayPIN, state);
  
  // Send confirmation to VC02
  String deviceName = devices[deviceId].deviceName;
  sendStateToVC02(deviceName, state);
  
  return true;
}

void handleFlipSwitches() {
  unsigned long actualMillis = millis();
  
  for (auto &flipSwitch : flipSwitches) {
    unsigned long lastFlipSwitchChange = flipSwitch.second.lastFlipSwitchChange;

    if (actualMillis - lastFlipSwitchChange > DEBOUNCE_TIME) {
      int flipSwitchPIN = flipSwitch.first;
      bool lastFlipSwitchState = flipSwitch.second.lastFlipSwitchState;
      bool activeLow = flipSwitch.second.activeLow;
      bool flipSwitchState = digitalRead(flipSwitchPIN);
      
      if(activeLow) flipSwitchState = !flipSwitchState;

      if (flipSwitchState != lastFlipSwitchState) {
#ifdef TACTILE_BUTTON
        if (flipSwitchState) {
#endif      
          flipSwitch.second.lastFlipSwitchChange = actualMillis;
          String deviceId = flipSwitch.second.deviceId;
          int relayPIN = devices[deviceId].relayPIN;
          bool newRelayState = !digitalRead(relayPIN);
          digitalWrite(relayPIN, newRelayState);

          Serial.printf("[FlipSwitch] Device %s: %s\r\n", 
                        deviceId.c_str(), newRelayState ? "ON" : "OFF");

          // Send to cloud if connected
          if(wifiConnected && sinricConnected) {
            SinricProSwitch &mySwitch = SinricPro[deviceId];
            mySwitch.sendPowerStateEvent(newRelayState);
          }
          
          // Send to VC02
          String deviceName = devices[deviceId].deviceName;
          sendStateToVC02(deviceName, newRelayState);
          
#ifdef TACTILE_BUTTON
        }
#endif      
        flipSwitch.second.lastFlipSwitchState = flipSwitchState;
      }
    }
  }
}

// ====== VC02 UART Communication ======
void handleVC02Commands() {
  while (VC02Serial.available()) {
    char c = VC02Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (uartBuffer.length() > 0) {
        processVC02Command(uartBuffer);
        uartBuffer = "";
      }
    } else {
      uartBuffer += c;
      
      // Process command if we have 4 characters (e.g., "AA11")
      if (uartBuffer.length() >= 4) {
        processVC02Command(uartBuffer);
        uartBuffer = "";
      }
    }
  }
}

void processVC02Command(String command) {
  command.trim();
  command.toUpperCase();
  
  Serial.printf("[VC02] Received: %s\r\n", command.c_str());
  
  String deviceName = "";
  bool newState = false;
  
  // Parse command
  if (command == "AA00") {
    deviceName = "LED";
    newState = false;
  } else if (command == "AA11") {
    deviceName = "LED";
    newState = true;
  } else if (command == "BB00") {
    deviceName = "FAN";
    newState = false;
  } else if (command == "BB11") {
    deviceName = "FAN";
    newState = true;
  } else {
    Serial.printf("[VC02] Unknown command: %s\r\n", command.c_str());
    return;
  }
  
  // Get device ID
  String deviceId = getDeviceIdByName(deviceName);
  if (deviceId == "") {
    Serial.printf("[VC02] Device not found: %s\r\n", deviceName.c_str());
    return;
  }
  
  // Control relay
  int relayPIN = devices[deviceId].relayPIN;
  digitalWrite(relayPIN, newState);
  
  Serial.printf("[VC02] %s turned %s\r\n", 
                deviceName.c_str(), newState ? "ON" : "OFF");
  
  // Send to cloud if connected
  if(wifiConnected && sinricConnected) {
    delay(RELAY_RESPONSE_DELAY);
    SinricProSwitch &mySwitch = SinricPro[deviceId];
    mySwitch.sendPowerStateEvent(newState);
  }
  
  // Send acknowledgment back to VC02
  VC02Serial.print("OK:");
  VC02Serial.println(command);
}

void sendStateToVC02(String deviceName, bool state) {
  String command = "";
  
  if (deviceName == "LED") {
    command = state ? "AA11" : "AA00";
  } else if (deviceName == "FAN") {
    command = state ? "BB11" : "BB00";
  }
  
  if (command != "") {
    VC02Serial.print("STATE:");
    VC02Serial.println(command);
    Serial.printf("[VC02] Sent state: %s\r\n", command.c_str());
  }
}

String getDeviceIdByName(String deviceName) {
  for (auto &device : devices) {
    if (device.second.deviceName == deviceName) {
      return device.first;
    }
  }
  return "";
}

// ====== Main Setup and Loop ======
void setup() {
  Serial.begin(BAUD_RATE);
  delay(500);
  
  Serial.println("\r\n========================================");
  Serial.println("ESP SinricPro + VC02 Voice Controller");
  Serial.println("========================================\r\n");
  
  setupRelays();
  setupFlipSwitches();
  setupVC02Serial();
  setupWiFi();
  setupSinricPro();
  
  Serial.println("\r\n[System] Setup complete! Ready for operation.\r\n");
}

void loop() {
  // Handle WiFi connection monitoring
  checkWiFiConnection();
  
  // Handle SinricPro only if WiFi connected
  if(wifiConnected) {
    SinricPro.handle();
  }
  
  // Handle manual switches (always works)
  handleFlipSwitches();
  
  // Handle VC02 UART commands (always works)
  handleVC02Commands();
  
  // Small delay to prevent WDT reset
  delay(1);
}