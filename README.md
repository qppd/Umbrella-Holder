# ☂️ Umbrella Holder - Smart Automated Umbrella Drying System

<div align="center">

![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)
![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg?style=for-the-badge)

*An intelligent microcontroller-based dryer for up to 6 umbrellas, featuring PID-controlled heating and automated blower operation via solid state relays. Designed for reliability and ease of use in public or private spaces.*

[🚀 Quick Start](#-quick-start) • [📖 Documentation](#-documentation) • [🛠️ Hardware](#️-hardware-requirements) • [💡 Features](#-features) • [🤝 Contributing](#-contributing)

</div>

---

## 🌟 Features

- ☂️ **Multi-Umbrella Capacity**: Dries 6 umbrellas simultaneously
- 🔥 **Smart Temperature Control**: PID-controlled heating system maintains optimal 60°C
- 💨 **Automated Airflow**: Dedicated blower relay for efficient moisture removal
- ⏱️ **Timed Operation**: 8-minute automatic drying cycle with safety shutoff
- 📺 **Real-time Display**: LCD shows current status, temperature, and humidity
- 🔘 **Tactile Controls**: 4 responsive buttons for user interaction
- 💡 **Visual Feedback**: LED indicators for system status
- 🌡️ **Environmental Monitoring**: DHT22 sensor for precise temperature & humidity tracking
- 🔌 **Dual Relay System**: Separate control for heater and blower operations
- 🧠 **Development Mode**: Built-in testing and debugging capabilities

---

## 🚀 Quick Start

### Prerequisites
- Arduino IDE (1.8.0 or higher)
- Required libraries (see [Dependencies](#-dependencies))
- Compatible Arduino board

### Installation
1. **Clone the repository**
   ```bash
   git clone https://github.com/qppd/Umbrella-Holder.git
   cd Umbrella-Holder
   ```

2. **Install Dependencies**
   - Open Arduino IDE
   - Go to `Sketch > Include Library > Manage Libraries`
   - Install the required libraries listed below

3. **Upload Code**
   - Open `UmbrellaDryer.ino` in Arduino IDE
   - Select your board and port
   - Upload the sketch

4. **Hardware Setup**
   - Connect components according to the [wiring diagram](#-wiring-diagram)
   - Power on the system

---

## 🛠️ Hardware Requirements

### Essential Components
| Component | Quantity | Purpose |
|-----------|----------|---------|
| Arduino-compatible board | 1 | Main controller |
| Solid State Relays (SSR) | 2 | Heater & blower control |
| DHT22 Sensor | 1 | Temperature/humidity monitoring |
| I2C LCD Display (20x4) | 1 | Status display |
| Tactile Push Buttons | 4 | User input |
| Status LEDs | 3 | Visual indicators |
| Heating Element | 1 | Umbrella drying |
| Blower Fan | 1 | Air circulation |

### Pin Configuration
```cpp
// Tactile Buttons
#define BUTTON_1 4    // Pin 4
#define BUTTON_2 5    // Pin 5  
#define BUTTON_3 6    // Pin 6
#define BUTTON_4 7    // Pin 7

// DHT22 Sensor
#define DHTPIN 10     // Pin 10

// Solid State Relays
#define RELAY_HEATER 8  // Pin 8 - Heater control
#define RELAY_BLOWER 9  // Pin 9 - Blower control

// LED Indicators
#define LED_1 2       // Pin 2 - Status LED 1
#define LED_2 3       // Pin 3 - Status LED 2  
#define LED_3 11      // Pin 11 - Status LED 3

// I2C Display (SDA/SCL pins)
// Address: 0x27
```

---

## 📂 Project Structure

```
UmbrellaDryer/
├── 📄 UmbrellaDryer.ino      # Main application logic & system control
├── 🌡️ DhtSensor.cpp/.h       # DHT22 temperature/humidity interface
├── 📺 I2cDisplay.cpp/.h      # LCD display management
├── 💡 LedIndicator.cpp/.h    # LED status indicator control
├── 🎛️ PidController.cpp/.h   # PID algorithm for heater control
├── 🔌 RelayModule.cpp/.h     # Solid state relay abstraction
├── 🔘 TactileButton.cpp/.h   # Button input handling & debouncing
├── 📌 Pins.h                 # Hardware pin definitions
└── 📖 README.md              # Project documentation
```

---

## ⚙️ System Operation

### 🔄 Operation Flow
1. **System Initialization**
   - All modules initialize (sensors, relays, display, LEDs)
   - LCD displays "Initialized!!!" message
   - System enters production mode

2. **Automatic Drying Cycle**
   - **Blower**: Activates immediately and runs for full 8-minute duration
   - **Heater**: PID-controlled to maintain 60°C ± 1°C
   - **LEDs**: All three indicators illuminate during operation
   - **Display**: Shows "Drying..." status and real-time data

3. **Temperature Control Logic**
   ```cpp
   if (temperature < 59°C) → Heater ON
   if (temperature > 61°C) → Heater OFF
   ```

4. **Cycle Completion**
   - After 8 minutes: All relays and LEDs turn OFF
   - LCD displays "Done!" message
   - System enters standby mode

### 🔧 Development Mode
Set `SYSTEM_MODE` to `MODE_DEVELOPMENT` for component testing via Serial Monitor at 115200 baud. The system provides a memory-optimized command interface for testing and debugging.

**Quick Start in Development Mode:**
1. Set `#define SYSTEM_MODE MODE_DEVELOPMENT` in UmbrellaDryer.ino
2. Upload to Arduino
3. Open Serial Monitor (115200 baud)
4. Type `help` to see all available commands

**Verified Working Components:**
- ✅ DHT22 Temperature/Humidity Sensor - **TESTED & WORKING**
- ✅ Relay Modules (Heater & Blower) - **TESTED & WORKING**
- ✅ I2C LCD Display - Ready for testing
- ✅ LED Indicators - Ready for testing
- ✅ PID Controller - Ready for testing
- ✅ Tactile Buttons - Ready for testing

**Key Features:**
- � Memory-optimized serial commands (fits in Arduino Uno RAM)
- 📊 Real-time sensor monitoring
- 🎛️ Manual hardware control
- 📈 System health status reporting
- 🛠️ Component-by-component testing

---

## 🖥️ Serial Commands Reference (Development Mode)

### System Information Commands
| Command | Description | Status |
|---------|-------------|--------|
| `help` | Display command reference | ✅ Available |
| `status` | Show component status overview | ✅ Available |
| `sensors` | Get current sensor readings | ✅ Available (DHT22 working) |

### Component Testing Commands (Memory Optimized)
| Command | Description | Test Status |
|---------|-------------|-------------|
| `test_dht` | Test DHT22 sensor | ✅ **WORKING** - Temperature/humidity readings |
| `test_lcd` | Test I2C LCD display | ✅ Available - Clear and write test |
| `test_led` | Test LED indicators | ✅ Available - Sequential LED test |
| `test_relay` | Test relay modules | ✅ **WORKING** - Heater and blower control |
| `test_button` | Test tactile buttons | ✅ Available - 5-second button test |
| `test_pid` | Test PID controller | ✅ Available - Output calculation test |

### Manual Hardware Control (Verified)
| Command | Description | Status |
|---------|-------------|--------|
| `h_on` / `h_off` | Control heater relay | ✅ **TESTED & WORKING** |
| `b_on` / `b_off` | Control blower relay | ✅ **TESTED & WORKING** |
| `led_on` / `led_off` | Control all LEDs | ✅ Available |
| `clear` | Clear LCD display | ✅ Available |

### System Control
| Command | Description | Action |
|---------|-------------|--------|
| `monitor` | Toggle continuous monitoring | ✅ Available |
| `reset` | Software reset | ✅ Available |

### Example Testing Session (Memory Optimized)
```
UMBRELLA DRYER - DEV MODE
Init... OK
Type 'help' for commands

> status
--- STATUS ---
DHT: OK
LCD: OK
LED: OK
RELAY: OK
BTN: OK
PID: OK
Monitor: OFF

> sensors
T:25.30C H:60.20%
PID:2500.00

> test_dht
DHT - T:25.30 H:60.20 OK

> test_relay
Relay OK

> h_on
Heater ON

> h_off
Heater OFF

> b_on
Blower ON

> b_off
Blower OFF
```

---

## 🧑‍💻 Code Architecture

### Core Classes

#### 🎛️ PidController
- **Purpose**: Maintains precise temperature control
- **Features**: Configurable PID parameters (Kp=4, Ki=0, Kd=22)
- **Methods**: `init()`, `compute()`, `setCurrentTemperature()`, `getOutput()`

#### 🌡️ DhtSensor  
- **Purpose**: Environmental monitoring
- **Features**: Temperature (°C/°F) and humidity readings
- **Methods**: `begin()`, `getTemperature()`, `getHumidity()`

#### 📺 I2cDisplay
- **Purpose**: User interface and status display
- **Features**: 20x4 character LCD with I2C communication
- **Methods**: `init()`, `clear()`, `setText()` (multiple overloads)

#### 🔌 RelayModule
- **Purpose**: High-power device control abstraction
- **Features**: SSR control for heater and blower
- **Methods**: `init()`, `set(relay, state)`

#### 💡 LedIndicator
- **Purpose**: Visual system status feedback
- **Features**: Independent control of 3 status LEDs
- **Methods**: `init()`, `set(ledPin, state)`

#### 🔘 TactileButton
- **Purpose**: User input handling with debouncing
- **Features**: 4-button support with 50ms debounce
- **Methods**: `init()`, `setInputFlags()`, `resolveInputFlags()`

---

## � Configuration

### Temperature Settings
```cpp
const double HEATER_SETPOINT = 60.0;  // Target temperature in Celsius
const unsigned long DRYING_DURATION = 8UL * 60UL * 1000UL;  // 8 minutes
```

### PID Tuning Parameters
```cpp
double kp = 4;   // Proportional gain
double ki = 0;   // Integral gain  
double kd = 22;  // Derivative gain
```

### System Modes
```cpp
#define MODE_DEVELOPMENT 1  // Serial testing & diagnostics mode
#define MODE_PRODUCTION 2   // Full automatic operation mode
#define SYSTEM_MODE MODE_DEVELOPMENT  // Current mode (set for testing)
```

#### Development Mode Features (Optimized for Arduino Uno)
- **🔧 Memory-Optimized Commands:** 15 essential commands for testing
- **📊 Real-time Sensor Data:** DHT22 temperature/humidity monitoring ✅ **WORKING**
- **🎛️ Hardware Control:** Direct relay, LED, and LCD control ✅ **RELAYS TESTED**
- **📈 Component Testing:** Individual module verification
- **🛠️ Manual Override:** Direct hardware control via serial
- **💾 RAM Efficient:** Optimized to fit Arduino Uno memory constraints
- **🔄 Status Reporting:** Compact system health monitoring
- **📋 Quick Testing:** Streamlined component verification

---

## 📦 Dependencies

Install these libraries through the Arduino Library Manager:

| Library | Version | Purpose |
|---------|---------|---------|
| [PID_v1](https://playground.arduino.cc/Code/PIDLibrary/) | ≥1.2.0 | Temperature control algorithm |
| [DHT sensor library](https://github.com/adafruit/DHT-sensor-library) | ≥1.4.0 | DHT22 sensor communication |
| [LiquidCrystal_I2C](https://github.com/johnrickman/LiquidCrystal_I2C) | ≥1.1.2 | I2C LCD display interface |

**Note:** Serial testing system optimized for Arduino Uno memory constraints using native Arduino functionality.

## 🧪 Testing Progress & Status

### ✅ **VERIFIED WORKING COMPONENTS**
| Component | Status | Test Command | Notes |
|-----------|--------|--------------|--------|
| DHT22 Sensor | ✅ **WORKING** | `test_dht` | Temperature & humidity readings confirmed |
| Heater Relay | ✅ **WORKING** | `h_on` / `h_off` | SSR control verified |
| Blower Relay | ✅ **WORKING** | `b_on` / `b_off` | SSR control verified |
| Serial Interface | ✅ **WORKING** | `help` | All commands responsive |

### 🔄 **COMPONENTS READY FOR TESTING**
| Component | Status | Test Command | Next Steps |
|-----------|--------|--------------|------------|
| I2C LCD Display | 🟡 Ready | `test_lcd` | Test display output |
| LED Indicators | 🟡 Ready | `test_led` | Test LED sequence |
| Tactile Buttons | 🟡 Ready | `test_button` | Test button inputs |
| PID Controller | 🟡 Ready | `test_pid` | Test with real sensor data |

### 📊 **System Health**
- **Memory Usage:** Optimized to fit Arduino Uno (< 75% RAM)
- **Code Size:** Reduced from 242% to ~60% of available memory
- **Core Functionality:** All essential testing features retained
- **Hardware Interface:** DHT22 and relays confirmed functional

### Installation Commands
```bash
# Using Arduino CLI
arduino-cli lib install "PID"
arduino-cli lib install "DHT sensor library"  
arduino-cli lib install "LiquidCrystal I2C"
```

---

## 🔌 Wiring Diagram

```
Arduino Uno/Nano    Component
================    =========
Digital Pin 2   →   LED 1 (Status)
Digital Pin 3   →   LED 2 (Status)  
Digital Pin 4   →   Button 1
Digital Pin 5   →   Button 2
Digital Pin 6   →   Button 3
Digital Pin 7   →   Button 4
Digital Pin 8   →   SSR Heater Control
Digital Pin 9   →   SSR Blower Control
Digital Pin 10  →   DHT22 Data Pin
Digital Pin 11  →   LED 3 (Status)
Analog Pin A4   →   LCD SDA (I2C)
Analog Pin A5   →   LCD SCL (I2C)
5V             →   VCC (All components)
GND            →   GND (All components)
```

---

## 🚦 Troubleshooting

### Common Issues

#### 🔥 Heater Not Maintaining Temperature
- Check SSR connections and power supply
- Verify DHT22 sensor readings with `test_dht`
- Adjust PID parameters if needed

#### 📺 LCD Not Displaying
- Verify I2C address (default: 0x27)
- Check SDA/SCL connections
- Test with `test_lcd` command

#### 🔘 Buttons Not Responsive  
- Check debounce delay settings
- Verify pin connections
- Use `test_button` for diagnostics

#### 💡 LEDs Not Working
- Verify LED polarity and current limiting resistors
- Test individual LEDs with `test_led`
- Check pin assignments in Pins.h

### 🔍 Development Mode Testing

#### Setting Up Serial Testing
1. **Enable Development Mode:**
   ```cpp
   #define SYSTEM_MODE MODE_DEVELOPMENT
   ```

2. **Serial Monitor Configuration:**
   - Baud Rate: 115200
   - Line Ending: Both NL & CR
   - Open Serial Monitor after upload

3. **First Steps:**
   ```
   > help          // Show all commands
   > status        // Check component health
   > test_all      // Run full system test
   ```

#### Systematic Testing Approach
1. **Component Verification:**
   ```
   > test_dht      // Verify sensor communication
   > test_lcd      // Check display functionality
   > test_led      // Validate LED indicators
   > test_relay    // Test relay switching
   ```

2. **Interactive Testing:**
   ```
   > interactive_on     // Enable button monitoring
   > monitor_start      // Start sensor logging
   ```

3. **Manual Control Testing:**
   ```
   > relay_heater_on    // Test heater control
   > led_all_on         // Test LED control
   > lcd_clear          // Test display control
   ```

#### Serial Command Troubleshooting

**✅ VERIFIED WORKING:**
- **DHT22 Sensor:** Temperature and humidity readings confirmed working
- **Relay Control:** Both heater and blower relays tested and functional
- **Serial Commands:** All testing commands responding correctly

**If you encounter issues:**
- **No Response:** Check baud rate (115200) and cable connection
- **Garbled Text:** Verify line ending settings (NL + CR)
- **Commands Not Working:** Type exact commands (case insensitive)
- **Memory Issues:** Current optimized version fits Arduino Uno constraints
- **Other Components:** Use individual test commands to verify LCD, LEDs, buttons

---

## 🤝 Contributing

We welcome contributions! Here's how you can help:

### 🐛 Bug Reports
- Use the [issue tracker](https://github.com/qppd/Umbrella-Holder/issues)
- Include detailed reproduction steps
- Provide system specifications

### 💡 Feature Requests  
- Check existing [issues](https://github.com/qppd/Umbrella-Holder/issues) first
- Describe the use case and benefits
- Consider backward compatibility

### 🔧 Pull Requests
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### 📋 Development Guidelines
- Follow Arduino coding standards
- Add comments for complex logic
- Test thoroughly before submitting
- Update documentation as needed

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🏷️ Tags

`#UmbrellaDryer` `#Arduino` `#PID` `#IoT` `#SmartHome` `#Automation` `#DHT22` `#I2C` `#SolidStateRelay` `#Maker` `#OpenSource` `#CPlusPlus` `#Embedded`

---

## 📬 Contact

📧 **Email**: quezon.province.pd@gmail.com  
🐙 **GitHub**: [github.com/qppd](https://github.com/qppd)  
🌐 **Portfolio**: [sajed-mendoza.onrender.com](https://sajed-mendoza.onrender.com)  
📘 **Facebook**: [facebook.com/qppd.dev](https://facebook.com/qppd.dev)  
📄 **Facebook Page**: [facebook.com/QUEZONPROVINCEDEVS](https://facebook.com/QUEZONPROVINCEDEVS)

---

## 👨‍💻 Author

**Created with ❤️ by [qppd](https://github.com/qppd)**

---

## 📊 GitHub Repository

[![Umbrella-Holder](https://img.shields.io/badge/GitHub-Umbrella--Holder-blue?style=for-the-badge&logo=github)](https://github.com/qppd/Umbrella-Holder)

*Transforming everyday appliances into intelligent solutions*

🌟 **Star this project if you find it useful!**

[![Stars](https://img.shields.io/github/stars/qppd/Umbrella-Holder?style=social)](https://github.com/qppd/Umbrella-Holder/stargazers)
[![Forks](https://img.shields.io/github/forks/qppd/Umbrella-Holder?style=social)](https://github.com/qppd/Umbrella-Holder/network/members)
[![Issues](https://img.shields.io/github/issues/qppd/Umbrella-Holder?style=social)](https://github.com/qppd/Umbrella-Holder/issues)

---

<div align="center">

### [⬆ Back to Top](#️-umbrella-dryer---smart-automated-drying-system)

</div>
