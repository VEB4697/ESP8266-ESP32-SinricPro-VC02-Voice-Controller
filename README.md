# ESP8266/ESP32 SinricPro + VC02 Voice Controller

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/Platform-ESP8266%20%7C%20ESP32-blue)](https://www.espressif.com/)
[![SinricPro](https://img.shields.io/badge/SinricPro-Compatible-green)](https://sinric.pro/)

A comprehensive IoT home automation system combining SinricPro cloud control with offline VC02 voice control, featuring WiFi stability management and multi-control interfaces.

## ✨ Features

- 🌐 **Cloud Control**: SinricPro integration with Alexa & Google Home
- 🎤 **Offline Voice Control**: VC02 voice controller with UART communication
- 🔘 **Manual Switches**: Physical button/toggle switch control
- 📡 **WiFi Stability**: Auto-reconnection with delayed response for stability
- ♻️ **State Synchronization**: All control methods sync across cloud
- 🔌 **Offline Mode**: Manual and voice controls work without internet
- 🛡️ **Error Handling**: Robust recovery from connection failures
- 📊 **Serial Debugging**: Comprehensive logging for troubleshooting

## 📋 Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Quick Start](#quick-start)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [API Reference](#api-reference)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## 🛠️ Hardware Requirements

### Essential Components
- ESP8266 (NodeMCU) or ESP32 development board
- VC02 Voice Recognition Module
- 2-Channel 5V Relay Module (with optocoupler)
- 2x Push buttons or toggle switches
- 5V 2A Power Supply
- Connecting wires and breadboard

### Optional
- Speaker (0.5W 8Ω) for VC02 audio feedback
- PCB for permanent installation
- Enclosure box

## 🚀 Quick Start

### 1. Clone Repository
```bash
git clone https://github.com/yourusername/esp-sinricpro-vc02.git
cd esp-sinricpro-vc02
```

### 2. Install Arduino IDE
Download from [Arduino.cc](https://www.arduino.cc/en/software)

### 3. Install Dependencies
```
Tools → Manage Libraries:
- SinricPro (by Boris Jaeger)
- ArduinoJson (v6.x)
- WebSockets (by Markus Sattler)
- EspSoftwareSerial (ESP8266 only)
```

### 4. Configure Credentials
Edit `config.h`:
```cpp
#define WIFI_SSID "YourWiFiName"
#define WIFI_PASS "YourPassword"
#define APP_KEY "your-sinricpro-app-key"
#define APP_SECRET "your-sinricpro-app-secret"
```

### 5. Upload and Run
```
1. Select Board: Tools → Board → NodeMCU 1.0
2. Select Port: Tools → Port → COMx
3. Click Upload button
4. Open Serial Monitor (115200 baud)
```

## 📦 Installation

### Step 1: Hardware Setup

#### ESP8266 Pin Connections
```
GPIO5 (D5)  → Relay IN1 (LED)
GPIO4 (D4)  → Relay IN2 (FAN)
GPIO14 (D14) → Switch 1 (LED)
GPIO12 (D12) → Switch 2 (FAN)
GPIO13 (D7) → VC02 RX
GPIO12 (D6) → VC02 TX
```

#### Power Connections
```
5V    → Relay VCC, VC02 VCC
GND   → Common Ground
VIN   → 5V (if not using USB)
```

### Step 2: Software Setup

#### Install ESP8266 Board
```
File → Preferences → Additional Board Manager URLs:
http://arduino.esp8266.com/stable/package_esp8266com_index.json

Tools → Board Manager → Install "ESP8266"
```

#### Install Required Libraries
All libraries available through Library Manager:
```cpp
#include <ESP8266WiFi.h>      // Core WiFi
#include <SinricPro.h>         // Cloud integration
#include <SinricProSwitch.h>   // Switch device
#include <SoftwareSerial.h>    // UART for VC02
```

### Step 3: SinricPro Setup

1. Create account at [sinric.pro](https://sinric.pro/)
2. Get your `APP_KEY` and `APP_SECRET` from Credentials
3. Create two Switch devices:
   - Device 1: "LED" or "Light"
   - Device 2: "FAN"
4. Copy Device IDs to code

## ⚙️ Configuration

### WiFi Settings
```cpp
#define WIFI_SSID "YourNetwork"
#define WIFI_PASS "YourPassword"
```

### Device Configuration
```cpp
std::map<String, deviceConfig_t> devices = {
    {"device_id_1", {5, 14, false, "LED"}},  // GPIO5, Switch GPIO14
    {"device_id_2", {4, 12, false, "FAN"}},  // GPIO4, Switch GPIO12
};
```

### Advanced Settings
```cpp
#define WIFI_CHECK_INTERVAL 5000     // WiFi check every 5s
#define MIN_WIFI_STRENGTH -70        // Minimum RSSI
#define RELAY_RESPONSE_DELAY 300     // Delay for stability (ms)
#define DEBOUNCE_TIME 250            // Switch debounce (ms)
#define VC02_BAUD_RATE 9600          // UART baud rate
```

## 📖 Usage

### Voice Commands (VC02)

The VC02 sends these UART commands:

| Voice Command | UART Code | Action |
|--------------|-----------|--------|
| "Turn on light" | `AA11` | LED ON |
| "Turn off light" | `AA00` | LED OFF |
| "Turn on fan" | `BB11` | FAN ON |
| "Turn off fan" | `BB00` | FAN OFF |

### Manual Control

Physical switches work independently:
- Press Switch 1 → Toggle LED
- Press Switch 2 → Toggle FAN
- States automatically sync to cloud

### Mobile App Control

**SinricPro App**:
1. Download from App Store/Google Play
2. Login with your account
3. Control devices from anywhere

**Alexa Integration**:
```
"Alexa, turn on LED"
"Alexa, turn off fan"
```

**Google Home Integration**:
```
"Hey Google, turn on light"
"Hey Google, turn off fan"
```

## 🔌 API Reference

### Main Functions

#### `onPowerState(String deviceId, bool &state)`
Handles device state changes from SinricPro cloud.
```cpp
bool onPowerState(String deviceId, bool &state) {
  // Applies relay response delay for WiFi stability
  // Controls relay
  // Sends confirmation to VC02
  return true;
}
```

#### `processVC02Command(String command)`
Processes UART commands from VC02 voice controller.
```cpp
void processVC02Command(String command) {
  // Parses command (AA11, BB00, etc.)
  // Controls relay
  // Syncs to cloud
  // Sends acknowledgment
}
```

#### `checkWiFiConnection()`
Monitors WiFi status and handles reconnection.
```cpp
void checkWiFiConnection() {
  // Checks connection every WIFI_CHECK_INTERVAL
  // Auto-reconnects if disconnected
  // Reinitializes SinricPro after reconnection
}
```

### UART Protocol

**Commands FROM VC02**:
```
AA11 - Turn LED ON
AA00 - Turn LED OFF
BB11 - Turn FAN ON
BB00 - Turn FAN OFF
```

**Responses TO VC02**:
```
OK:AA11      - Command acknowledged
STATE:BB00   - State update notification
```

## 🔧 Troubleshooting

### Common Issues

#### Upload Fails
```
✓ Hold FLASH button while uploading
✓ Check USB cable supports data transfer
✓ Try different USB port
✓ Reduce upload speed to 115200
✓ Install CH340 drivers
```

#### WiFi Won't Connect
```
✓ Verify 2.4GHz network (not 5GHz)
✓ Check SSID/password spelling
✓ Move closer to router
✓ Check Serial Monitor for errors
```

#### VC02 Not Responding
```
✓ Verify TX→RX and RX→TX crossed correctly
✓ Check baud rate (9600)
✓ Ensure VC02 has power
✓ Verify voice commands programmed
```

#### Relays Not Clicking
```
✓ Check 5V power to relay module
✓ Verify IN1/IN2 connections
✓ Test with multimeter
✓ Check relay LED indicators
```

### Debug Mode

Enable detailed logging:
```cpp
#define ENABLE_DEBUG 1
```

Serial Monitor output format:
```
[WiFi] Connection status
[SinricPro] Cloud events
[VC02] UART communication
[FlipSwitch] Manual switch events
[System] General status
```

## 📁 Project Structure

```
esp-sinricpro-vc02/
├── esp-sinricpro-vc02.ino    # Main Arduino sketch
├── config.h                   # Configuration file
├── README.md                  # This file
├── LICENSE                    # MIT License
├── docs/
│   ├── INSTALLATION.md        # Detailed installation guide
│   ├── CIRCUIT_DIAGRAM.png    # Wiring diagram
│   ├── PINOUT.md             # Pin configuration
│   └── API.md                # API documentation
├── examples/
│   ├── basic_test/           # Basic functionality test
│   ├── uart_test/            # VC02 UART testing
│   └── wifi_test/            # WiFi stability test
└── tools/
    ├── vc02_config/          # VC02 configuration utility
    └── serial_monitor/       # Custom serial monitor
```

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

### Development Setup
```bash
git clone https://github.com/yourusername/esp-sinricpro-vc02.git
cd esp-sinricpro-vc02
git checkout -b feature/your-feature
# Make changes
git commit -am "Add new feature"
git push origin feature/your-feature
```

### Code Style
- Follow Arduino style guide
- Comment complex logic
- Use descriptive variable names
- Test on both ESP8266 and ESP32

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- [SinricPro](https://sinric.pro/) for cloud integration platform
- [ESP8266 Community](https://github.com/esp8266/Arduino) for Arduino core
- VC02 manufacturers for voice recognition module
- Arduino community for libraries and support

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/yourusername/esp-sinricpro-vc02/issues)
- **Discussions**: [GitHub Discussions](https://github.com/yourusername/esp-sinricpro-vc02/discussions)
- **Email**: your.email@example.com

## 📊 Changelog

### Version 1.0.0 (2025-11-28)
- ✨ Initial release
- 🌐 SinricPro cloud integration
- 🎤 VC02 voice control support
- 📡 WiFi stability management
- 🔘 Manual switch control
- ♻️ Multi-source state synchronization

---

**Made with ❤️ for Home Automation Enthusiasts**

⭐ Star this repo if you find it helpful!