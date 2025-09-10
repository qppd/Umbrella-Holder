# ☂️ Umbrella Dryer

A microcontroller-based dryer for up to 6 umbrellas, featuring PID-controlled heating and blower operation via solid state relays. Designed for reliability and ease of use in public or private spaces.

---

## 🚀 Features
- **Dries 6 umbrellas simultaneously**
- **2 Solid State Relays**: One for heater (PID controlled), one for blower
- **PID Heater Control**: Maintains 60°C setpoint
- **8-Minute Drying Cycle**: Automatic shutoff
- **LCD Display**: Status updates
- **Tactile Buttons**: User input
- **LED Indicators**: Visual feedback
- **DHT22 Sensor**: Temperature & humidity monitoring

---

## 🛠️ Hardware Requirements
- Arduino-compatible board
- 2x Solid State Relays (SSR)
- DHT22 Temperature/Humidity Sensor
- LCD Display (I2C)
- Tactile Buttons (x4)
- LEDs (x3)

---

## 📂 File Structure
```
UmbrellaDryer.ino         # Main application logic
DhtSensor.cpp/.h         # DHT22 sensor interface
I2cDisplay.cpp/.h        # LCD display interface
LedIndicator.cpp/.h      # LED control
PidController.cpp/.h     # PID logic for heater
RelayModule.cpp/.h       # Relay control
TactileButton.cpp/.h     # Button handling
```

---

## ⚙️ How It Works
1. **Startup**: Initializes all modules (sensors, relays, display, etc.)
2. **Drying Cycle**: On boot, starts an 8-minute drying cycle:
   - Blower relay ON for full duration
   - Heater relay PID controlled at 60°C (ON below 59°C, OFF above 61°C)
   - LEDs indicate active drying
   - LCD displays status
3. **Shutdown**: After 8 minutes, all relays and LEDs turn OFF, LCD shows "Done!"

---

## 🧑‍💻 Code Overview
- **`UmbrellaDryer.ino`**: Main loop, production logic, and system initialization
- **`PidController`**: Uses Arduino PID library for heater control
- **`RelayModule`**: Abstraction for SSRs
- **`DhtSensor`**: Reads temperature/humidity
- **`I2cDisplay`**: Handles LCD output
- **`LedIndicator`**: Manages status LEDs
- **`TactileButton`**: Handles button input and debouncing

---

## 📝 Usage
1. Wire up hardware as per requirements
2. Upload code to Arduino
3. Power on: drying cycle starts automatically
4. Monitor status via LCD and LEDs

---

## 📦 Dependencies
- [PID_v1](https://playground.arduino.cc/Code/PIDLibrary/)
- [DHT sensor library](https://github.com/adafruit/DHT-sensor-library)
- [LiquidCrystal_I2C](https://github.com/johnrickman/LiquidCrystal_I2C)

---

## 🏷️ Tags
`#UmbrellaDryer` `#Arduino` `#PID` `#Relay` `#DHT22` `#I2C` `#Maker` `#OpenSource`

---

## 📬 Contact
📧 Email: quezon.province.pd@gmail.com  
🐙 GitHub: github.com/qppd  
🌐 Portfolio: sajed-mendoza.onrender.com  
📘 Facebook: facebook.com/qppd.dev  
📄 Facebook Page: facebook.com/QUEZONPROVINCEDEVS
