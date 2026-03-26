# Ethanol Content Analyzer (ESP32-C3) work in progress, not tested yet

Open source ethanol content analyzer that reads signals from Continental Flex Fuel sensors and reports ethanol percentage and fuel temperature via serial output.

Based on the [original project](https://github.com/outlandnish/fw-ethanol-content-analyzer) which targeted the Arduino Nano Every (ATmega4809).

### Features
- **Ethanol content** — reads sensor frequency (50–150 Hz) and converts to ethanol percentage (0–100%)
- **Fuel temperature** — decodes the duty cycle to report fuel temperature (-40°C to 125°C)
- **Serial output** — prints all readings to USB serial at 115200 baud
- **WiFi / BLE capable** — ESP32-C3 opens up future possibilities for wireless data logging

### Hardware

**MCU**: ESP32-C3 Super Mini

**Input circuit** (5V sensor → 3.3V GPIO):
- Voltage divider on the sensor signal line (10kΩ + 20kΩ) to step down the 5V square wave to ~3.3V
- Connect the divided signal to **GPIO4**

**Power**: USB-C (easiest) or 5V regulator from vehicle 12V

### Pin Map

| Function | GPIO | Notes |
|---|---|---|
| Sensor Input | GPIO4 | Use voltage divider from 5V signal |

### Parts List
- 1x ESP32-C3 Super Mini
- 1x Continental Flex Fuel Sensor (GM 13577429 or similar)
- 1x 10kΩ resistor (voltage divider)
- 1x 20kΩ resistor (voltage divider)
- 1x USB-C cable

### Building

Built with [PlatformIO](https://platformio.org/). Open the project folder and build for the `esp32c3` environment.

### Serial Output

At 115200 baud you'll see:
```
Ethanol Content Analyzer - ESP32-C3
Waiting for sensor data...
Period (us): 12500  Frequency: 80.0 Hz  Ethanol: 30.0%  Duty Cycle: 50.0%  Fuel Temp: 22.5 C
```

### Sensor Details

Uses Continental Flex Fuel sensor (e.g., GM 13577429):
- **Frequency** (50–150 Hz) → Ethanol content (0–100%)
- **Duty cycle** (10–90%) → Fuel temperature (-40°C to 125°C)
- **180–190 Hz** → Contaminated fuel
