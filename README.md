# ☂️ Umbrella Dryer - Smart Automated Drying System

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

### � Development Mode
Toggle `SYSTEM_MODE` to `MODE_DEVELOPMENT` for testing individual components:
- `test_button` - Test button responsiveness
- `test_dht` - Check sensor readings  
- `test_lcd` - Verify display functionality
- `test_led` - Test LED indicators
- `test_pid` - Monitor PID controller
- `test_relay` - Test relay operations

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
#define MODE_DEVELOPMENT 1  // Component testing mode
#define MODE_PRODUCTION 2   // Full system operation
#define SYSTEM_MODE MODE_PRODUCTION  // Current mode
```

---

## 📦 Dependencies

Install these libraries through the Arduino Library Manager:

| Library | Version | Purpose |
|---------|---------|---------|
| [PID_v1](https://playground.arduino.cc/Code/PIDLibrary/) | ≥1.2.0 | Temperature control algorithm |
| [DHT sensor library](https://github.com/adafruit/DHT-sensor-library) | ≥1.4.0 | DHT22 sensor communication |
| [LiquidCrystal_I2C](https://github.com/johnrickman/LiquidCrystal_I2C) | ≥1.1.2 | I2C LCD display interface |

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

### Debug Mode
Switch to development mode for component-level testing:
```cpp
#define SYSTEM_MODE MODE_DEVELOPMENT
```

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
