#pragma once

#include "Arduino.h"
#include "driver/twai.h"
#include "esp_task_wdt.h"

// I/O
#define ECA_INPUT                 4             // Flex fuel sensor signal input (use voltage divider from 5V!)
#define CAN_TX_PIN                5             // TWAI TX -> CAN transceiver TX
#define CAN_RX_PIN                6             // TWAI RX -> CAN transceiver RX

// constants
#define FREQUENCY_ALPHA           0.01f
#define MAX_FREQUENCY             200.f         // Hz

#define E0_FREQUENCY              50.f          // Hz
#define E100_FREQUENCY            150.f         // Hz
#define ETHANOL_FREQUENCY_SCALER  ((E100_FREQUENCY - E0_FREQUENCY) / 100.0f)

// Signal validation thresholds (with hysteresis margin per spec)
#define FREQ_UNDERRANGE_LIMIT     45.f          // Hz — below this = sensor failure / wiring short
#define FREQ_OVERRANGE_LIMIT      155.f         // Hz — above this = water contamination

// Pulse width validation (duty cycle boundaries)
#define DUTY_CYCLE_MIN            5.0f          // % — below this = sensor disconnected
#define DUTY_CYCLE_MAX            95.0f         // % — above this = sensor disconnected

// Safe default values sent on CAN during error conditions
#define SAFE_ETHANOL_DEFAULT      30.0f         // % — Brazilian gasoline (gasolina comum ~E27)
#define SAFE_TEMP_DEFAULT         20.0f         // °C — room temperature default

// Startup stabilization
#define STABLE_PULSES_REQUIRED    3             // Wait for N valid readings before CAN output

// Watchdog
#define WDT_TIMEOUT_S             5             // Watchdog timeout in seconds

// Temperature
#define TEMP_MIN                  -40.0f        // °C
#define TEMP_MAX                  125.0f        // °C

// Zeitronix ECA-2 CAN Bus defaults
#define ZEITRONIX_CAN_ID          0x00EC        // Default standard 11-bit CAN ID
#define ZEITRONIX_CAN_SPEED       TWAI_TIMING_CONFIG_500KBITS()  // 500 Kbps
#define ZEITRONIX_CAN_INTERVAL_MS 250           // 4 Hz update rate
#define ZEITRONIX_SENSOR_OK       0x00
#define ZEITRONIX_SENSOR_FAULT    0x01

// Sensor timeout
#define SENSOR_TIMEOUT_MS         500           // No updates for 500ms => FAULT

// Sensor state enumeration
enum SensorState {
  SENSOR_INITIALIZING,    // Waiting for stable pulses after boot
  SENSOR_OK,              // Valid readings within normal range
  SENSOR_UNDERRANGE,      // Frequency < 45Hz — sensor failure / wiring
  SENSOR_CONTAMINATED,    // Frequency > 155Hz — water in fuel
  SENSOR_DUTY_INVALID,    // Pulse width out of valid range — disconnected
  SENSOR_TIMEOUT          // No data received within timeout window
};

// State variables
extern float ethanol;
extern float fuelTemperature;
extern bool canReady;
extern uint32_t lastSensorUpdateMs;
extern SensorState sensorState;
extern uint8_t stablePulseCount;

// Frequency and duty cycle readings
extern float frequency, dutyCycle;
extern const float frequencyScaler;

// ISR shared variables
extern volatile uint32_t risingEdgeTime;
extern volatile uint32_t fallingEdgeTime;
extern volatile uint32_t period;
extern volatile float rawDutyCycle;
extern volatile bool newData;

// Function declarations
bool calculateFrequency();
SensorState validateSignal(float freq, float duty);
void frequencyToEthanolContent(float frequency, float scaler);
void dutyCycleToFuelTemperature(float dutyCycle);

void IRAM_ATTR onSensorEdge();

void initCAN();
void sendZeitronixCANMessage();
