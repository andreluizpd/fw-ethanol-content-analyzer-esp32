# Ethanol Content Analyzer (ESP32-C3) work in progress, not tested yet

[![Made with AI](https://img.shields.io/badge/Made%20with-AI-lightgrey?style=for-the-badge)](https://github.com/mefengl/made-by-ai)

Open source ethanol content analyzer that reads signals from Continental Flex Fuel sensors and reports ethanol percentage and fuel temperature via serial and CAN bus output. CAN messages are compatible with the **Zeitronix ECA-2 CAN Bus** protocol.

Based on the [original project](https://github.com/outlandnish/fw-ethanol-content-analyzer) which targeted the Arduino Nano Every (ATmega4809).

### Features

- **Ethanol content** — reads sensor frequency (50–150 Hz) and converts to ethanol percentage (0–100%)
- **Fuel temperature** — decodes the duty cycle to report fuel temperature (-40°C to 125°C)
- **CAN bus output** — sends Zeitronix ECA-2 compatible CAN messages (500 Kbps, 4 Hz)
- **Serial output** — prints all readings to USB serial at 115200 baud
- **WiFi / BLE capable** — ESP32-C3 opens up future possibilities for wireless data logging

### Hardware

**MCU**: ESP32-C3 Super Mini

**Input circuit** (5V sensor → 3.3V GPIO):

- Voltage divider on the sensor signal line (10kΩ + 20kΩ) to step down the 5V square wave to ~3.3V
- Connect the divided signal to **GPIO4**

**CAN bus** — requires an external 3.3V CAN transceiver (e.g. SN65HVD230):

- Connect **GPIO5** (TX) and **GPIO6** (RX) to the transceiver

**Power**: USB-C (easiest) or 5V regulator from vehicle 12V

### Pin Map

| Function     | GPIO  | Notes                              |
| ------------ | ----- | ---------------------------------- |
| Sensor Input | GPIO4 | Use voltage divider from 5V signal |
| CAN TX       | GPIO5 | To CAN transceiver TX pin          |
| CAN RX       | GPIO6 | From CAN transceiver RX pin        |

### Parts List

- 1x ESP32-C3 Super Mini
- 1x Continental Flex Fuel Sensor (GM 13577429 or similar)
- 1x 3.3V CAN transceiver (SN65HVD230 or similar)
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

### CAN Bus Output (Zeitronix ECA-2 Compatible)

Messages are sent using the Zeitronix ECA-2 CAN protocol so that existing dashes, ECUs, and loggers that support the Zeitronix ECA work out of the box.

| Setting     | Default                    |
| ----------- | -------------------------- |
| CAN ID      | `0x00EC` (standard 11-bit) |
| Speed       | 500 Kbps                   |
| Update rate | 250 ms (4 Hz)              |

| Byte | Content       | Encoding                |
| ---- | ------------- | ----------------------- |
| 0    | Ethanol %     | 0–100, direct value     |
| 1    | Fuel Temp °C  | raw byte − 40           |
| 2–6  | Reserved      | 0x00                    |
| 7    | Sensor Status | 0x00 = OK, 0x01 = fault |

### Sensor Details

Uses Continental Flex Fuel sensor (e.g., GM 13577429):

- **Frequency** (50–150 Hz) → Ethanol content (0–100%)
- **Duty cycle** (10–90%) → Fuel temperature (-40°C to 125°C)
- **180–190 Hz** → Contaminated fuel
