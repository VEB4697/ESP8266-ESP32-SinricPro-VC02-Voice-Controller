/*
 * ESP8266/ESP32 SinricPro Multi-Switch with VC02 Voice Controller Integration
 * 
 * Hardware Configuration:
 * - ACTIVE-LOW Relays: LOW = ON, HIGH = OFF
 * - ACTIVE-HIGH Touch Sensors: HIGH = Touched, LOW = Not Touched
 * 
 * Pin Configuration (ESP8266):
 * - Touch Sensor 1 (LED): D1 (GPIO5)
 * - Touch Sensor 2 (FAN): D2 (GPIO4)
 * - Relay 1 (LED): D5 (GPIO14) - Active LOW
 * - Relay 2 (FAN): D8 (GPIO15) - Active LOW
 * - VC02 RX: D6 (GPIO12)
 * - VC02 TX: D7 (GPIO13)
 * 
 * UART Commands from VC02:
 * - AA00 = LED OFF, AA11 = LED ON
 * - BB00 = FAN OFF, BB11 = FAN ON
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
  #define VC02_RX_PIN 12  // D6 -> GPIO12 - Connect to VC02 TX
  #define VC02_TX_PIN 3  // RX -> GPIO13 - Connect to VC02 RX
  SoftwareSerial VC02Serial(VC02_RX_PIN, VC02_TX_PIN);
#elif defined(ESP32)
  #define VC02_RX_PIN 16
  #define VC02_TX_PIN 17
  HardwareSerial VC02Serial(2);
#endif

#define VC02_BAUD_RATE 9600

// ====== Pin Configuration (ESP8266 NodeMCU) ======
// Touch Sensors (Active HIGH - 3.3V when touched)
#define TOUCH_SENSOR_LED  5   // D1 -> GPIO5
#define TOUCH_SENSOR_FAN  4   // D2 -> GPIO4

// Relays (Active LOW - GND to turn ON)
#define RELAY_LED        14   // D5 -> GPIO14
#define RELAY_FAN        13   // D7 -> GPIO13

// ====== System Configuration ======
#define BAUD_RATE            115200
#define DEBOUNCE_TIME        250
#define WIFI_CHECK_INTERVAL  5000
#define MIN_WIFI_STRENGTH    -70
#define RELAY_RESPONSE_DELAY 300

// Device configuration structure
typedef struct {
  int relayPIN;
  int touchSensorPIN;
  String deviceName;
} deviceConfig_t;

// Device mapping with SinricPro Device IDs
std::map<String, deviceConfig_t> devices = {
    // {deviceId, {relayPIN, touchSensorPIN, deviceName}}
    {"688c44e8ddd2551252bb6f0f", {RELAY_LED, TOUCH_SENSOR_LED, "LED"}},
    {"688c44acedeca866fe97c5f1", {RELAY_FAN, TOUCH_SENSOR_FAN, "FAN"}},
};

// Touch sensor state tracking
typedef struct {
  String deviceId;
  bool lastTouchState;
  unsigned long lastTouchChange;
} touchSensorConfig_t;

std::map<int, touchSensorConfig_t> touchSensors;

// System status variables
unsigned long lastWiFiCheck = 0;
bool wifiConnected = false;
bool sinricConnected = false;
String uartBuffer = "";
uint16_t receivedValue = 0;  // Store combined hex value from VC02

// ====== Function Prototypes ======
void setupRelays();
void setupTouchSensors();
void setupWiFi();
void setupSinricPro();
void setupVC02Serial();
void checkWiFiConnection();
void handleTouchSensors();
void handleVC02Commands();
void processVC02Command(uint16_t hexCommand);
void sendStateToVC02(String deviceName, bool state);
void setRelayState(int relayPIN, bool state);
bool getRelayState(int relayPIN);
bool onPowerState(String deviceId, bool &state);
String getDeviceIdByName(String deviceName);

// ====== Relay Control Functions (Active LOW Logic) ======
void setRelayState(int relayPIN, bool state) {
  // Active LOW relay: LOW = ON, HIGH = OFF
  // state: true = ON, false = OFF
  if (state) {
    digitalWrite(relayPIN, LOW);   // LOW = Relay ON (GND)
  } else {
    digitalWrite(relayPIN, HIGH);  // HIGH = Relay OFF (3.3V)
  }
}

bool getRelayState(int relayPIN) {
  // Read current relay state
  // Active LOW: LOW = ON (true), HIGH = OFF (false)
  return (digitalRead(relayPIN) == LOW);
}

// ====== Setup Functions ======
void setupRelays() { 
  Serial.println("[Setup] Initializing relays (Active LOW)...");
  
  for (auto &device : devices) {
    int relayPIN = device.second.relayPIN;
    pinMode(relayPIN, OUTPUT);
    setRelayState(relayPIN, false); // Initialize all relays to OFF
    
    Serial.printf("  Relay %s (Pin %d): OFF\r\n", 
                  device.second.deviceName.c_str(), relayPIN);
  }
  
  Serial.println("[Setup] Relays initialized successfully");
}

void setupTouchSensors() {
  Serial.println("[Setup] Initializing touch sensors (Active HIGH)...");
  
  for (auto &device : devices) {
    touchSensorConfig_t sensorConfig;
    sensorConfig.deviceId = device.first;
    sensorConfig.lastTouchChange = 0;
    sensorConfig.lastTouchState = false;
    
    int sensorPIN = device.second.touchSensorPIN;
    pinMode(sensorPIN, INPUT);  // Touch sensors are active HIGH
    
    touchSensors[sensorPIN] = sensorConfig;
    
    Serial.printf("  Touch Sensor %s (Pin %d): Ready\r\n", 
                  device.second.deviceName.c_str(), sensorPIN);
  }
  
  Serial.println("[Setup] Touch sensors initialized successfully");
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
  Serial.printf("[VC02] RX: GPIO%d (D6), TX: GPIO%d (D7), Baud: %d\r\n", 
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
      int32_t rssi = WiFi.RSSI();
      
      if(!wifiConnected) {
        wifiConnected = true;
        Serial.printf("[WiFi] Reconnected! IP: %s\r\n", WiFi.localIP().toString().c_str());
        
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
  if(wifiConnected) {
    delay(RELAY_RESPONSE_DELAY);
  }
  
  Serial.printf("[SinricPro] %s: %s\r\n", 
                devices[deviceId].deviceName.c_str(), 
                state ? "ON" : "OFF");
  
  int relayPIN = devices[deviceId].relayPIN;
  setRelayState(relayPIN, state);
  
  String deviceName = devices[deviceId].deviceName;
  sendStateToVC02(deviceName, state);
  
  return true;
}

// ====== Touch Sensor Handler (Active HIGH, Toggle behavior) ======
void handleTouchSensors() {
  unsigned long currentMillis = millis();
  
  for (auto &sensor : touchSensors) {
    int sensorPIN = sensor.first;
    touchSensorConfig_t &config = sensor.second;
    
    // Check debounce time
    if (currentMillis - config.lastTouchChange < DEBOUNCE_TIME) {
      continue;
    }
    
    // Read touch sensor (Active HIGH - HIGH when touched)
    bool currentTouchState = digitalRead(sensorPIN);
    
    // Detect touch (transition from LOW to HIGH)
    if (currentTouchState == HIGH && config.lastTouchState == LOW) {
      config.lastTouchChange = currentMillis;
      config.lastTouchState = currentTouchState;
      
      // Toggle relay on touch
      String deviceId = config.deviceId;
      int relayPIN = devices[deviceId].relayPIN;
      
      // Get current relay state and toggle it
      bool currentRelayState = getRelayState(relayPIN);
      bool newRelayState = !currentRelayState;
      
      setRelayState(relayPIN, newRelayState);
      
      Serial.printf("[Touch] %s sensor touched -> %s\r\n", 
                    devices[deviceId].deviceName.c_str(),
                    newRelayState ? "ON" : "OFF");
      
      // Sync to cloud if connected
      if(wifiConnected && sinricConnected) {
        SinricProSwitch &mySwitch = SinricPro[deviceId];
        mySwitch.sendPowerStateEvent(newRelayState);
      }
      
      // Send to VC02
      sendStateToVC02(devices[deviceId].deviceName, newRelayState);
    }
    
    // Update last state when touch is released (HIGH to LOW)
    if (currentTouchState == LOW && config.lastTouchState == HIGH) {
      config.lastTouchState = currentTouchState;
    }
  }
}

// ====== VC02 UART Communication (Binary Hex Format) ======
void handleVC02Commands() {
  if (VC02Serial.available() >= 2) {
    // Read two bytes from VC02
    byte highByte = VC02Serial.read();
    byte lowByte = VC02Serial.read();
    
    // Combine the two bytes into a single 16-bit hex value
    receivedValue = (highByte << 8) | lowByte;
    
    // Print the received value in HEX format
    Serial.print("[VC02] Received HEX value: 0x");
    Serial.println(receivedValue, HEX);
    
    // Process the hex command
    processVC02Command(receivedValue);
  }
}

void processVC02Command(uint16_t hexCommand) {
  String deviceName = "";
  bool newState = false;
  bool validCommand = false;
  
  // Parse VC02 hex commands using switch statement
  switch(hexCommand) {
    case 0xAA00:  // LED OFF
      deviceName = "LED";
      newState = false;
      validCommand = true;
      break;
      
    case 0xAA11:  // LED ON
      deviceName = "LED";
      newState = true;
      validCommand = true;
      break;
      
    case 0xBB00:  // FAN OFF
      deviceName = "FAN";
      newState = false;
      validCommand = true;
      break;
      
    case 0xBB11:  // FAN ON
      deviceName = "FAN";
      newState = true;
      validCommand = true;
      break;
      
    default:
      Serial.printf("[VC02] Unknown HEX command: 0x%04X\r\n", hexCommand);
      return;
  }
  
  if(!validCommand) {
    return;
  }
  
  // Get device ID and control relay
  String deviceId = getDeviceIdByName(deviceName);
  if (deviceId == "") {
    Serial.printf("[VC02] Device not found: %s\r\n", deviceName.c_str());
    return;
  }
  
  int relayPIN = devices[deviceId].relayPIN;
  setRelayState(relayPIN, newState);
  
  Serial.printf("[VC02] %s turned %s (0x%04X)\r\n", 
                deviceName.c_str(), newState ? "ON" : "OFF", hexCommand);
  
  // Sync to cloud if connected
  if(wifiConnected && sinricConnected) {
    delay(RELAY_RESPONSE_DELAY);
    SinricProSwitch &mySwitch = SinricPro[deviceId];
    mySwitch.sendPowerStateEvent(newState);
  }
  
  // Send acknowledgment with hex value
  VC02Serial.print("OK:0x");
  VC02Serial.println(hexCommand, HEX);
  
  // Reset received value
  receivedValue = 0;
}

void sendStateToVC02(String deviceName, bool state) {
  uint16_t hexValue = 0;
  
  // Convert device state to hex value
  if (deviceName == "LED") {
    hexValue = state ? 0xAA11 : 0xAA00;
  } else if (deviceName == "FAN") {
    hexValue = state ? 0xBB11 : 0xBB00;
  }
  
  if (hexValue != 0) {
    // Send hex value as two bytes
    byte highByte = (hexValue >> 8) & 0xFF;
    byte lowByte = hexValue & 0xFF;
    
    VC02Serial.write(highByte);
    VC02Serial.write(lowByte);
    
    Serial.printf("[VC02] Sent state: 0x%04X\r\n", hexValue);
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
  Serial.println("ESP8266 Home Automation System");
  Serial.println("Active-LOW Relays | Active-HIGH Touch");
  Serial.println("========================================\r\n");
  
  setupRelays();
  setupTouchSensors();
  setupVC02Serial();
  setupWiFi();
  setupSinricPro();
  
  Serial.println("\r\n========================================");
  Serial.println("[System] Initialization Complete!");
  Serial.println("[System] Ready for operation");
  Serial.println("========================================\r\n");
  
  // Display pin configuration
  Serial.println("[Info] Pin Configuration:");
  Serial.println("  Touch Sensors:");
  Serial.printf("    LED: D1 (GPIO%d)\r\n", TOUCH_SENSOR_LED);
  Serial.printf("    FAN: D2 (GPIO%d)\r\n", TOUCH_SENSOR_FAN);
  Serial.println("  Relays (Active LOW):");
  Serial.printf("    LED: D5 (GPIO%d)\r\n", RELAY_LED);
  Serial.printf("    FAN: D7 (GPIO%d)\r\n", RELAY_FAN);
  Serial.println("  VC02 UART:");
  Serial.printf("    RX: D6 (GPIO%d)\r\n", VC02_RX_PIN);
  Serial.printf("    TX: RX (GPIO%d)\r\n", VC02_TX_PIN);
  Serial.println("========================================\r\n");
}

void loop() {
  // WiFi connection monitoring
  checkWiFiConnection();
  
  // SinricPro cloud handling
  if(wifiConnected) {
    SinricPro.handle();
  }
  
  // Touch sensor monitoring (always active)
  handleTouchSensors();
  
  // VC02 UART command handling (always active)
  handleVC02Commands();
  
  // Prevent WDT reset
  delay(1);
}