#pragma once

#include "Arduino.h"
#include "driver/twai.h"
#include "esp_task_wdt.h"

// I/O
#define ECA_INPUT                 4
#define CAN_TX_PIN                5
#define CAN_RX_PIN                6

// Signal processing
#define FREQUENCY_ALPHA           0.01f
#define TEMPERATURE_ALPHA         0.005f
#define MAX_FREQUENCY             200.f

#define E0_FREQUENCY              50.f
#define E100_FREQUENCY            150.f
#define ETHANOL_FREQUENCY_SCALER  ((E100_FREQUENCY - E0_FREQUENCY) / 100.0f)

// Validation thresholds
#define FREQ_UNDERRANGE_LIMIT     45.f
#define FREQ_OVERRANGE_LIMIT      155.f
#define PULSE_WIDTH_MIN_US        900
#define PULSE_WIDTH_MAX_US        5100
#define PULSE_WIDTH_LOW_US        1000
#define PULSE_WIDTH_HIGH_US       5000

// Safe fallback values
#define SAFE_ETHANOL_DEFAULT      30.0f
#define SAFE_TEMP_DEFAULT         20.0f

// Startup stabilization
#define STABLE_PULSES_REQUIRED    3

// Watchdog / timeout
#define WDT_TIMEOUT_S             5
#define SENSOR_TIMEOUT_MS         500

// Temperature
#define TEMP_MIN                  -40.0f
#define TEMP_MAX                  125.0f

// Zeitronix ECA-2 CAN Bus defaults
#define ZEITRONIX_CAN_ID          0x00EC
#define ZEITRONIX_CAN_SPEED       TWAI_TIMING_CONFIG_500KBITS()
#define ZEITRONIX_CAN_INTERVAL_MS 250
#define ZEITRONIX_SENSOR_OK       0x00
#define ZEITRONIX_SENSOR_FAULT    0x01

enum SensorState {
  SENSOR_INITIALIZING,
  SENSOR_OK,
  SENSOR_UNDERRANGE,
  SENSOR_CONTAMINATED,
  SENSOR_PULSE_INVALID,
  SENSOR_TIMEOUT
};

extern float ethanol;
extern float fuelTemperature;
extern bool canReady;
extern uint32_t lastSensorUpdateMs;
extern SensorState sensorState;
extern uint8_t stablePulseCount;

extern float frequency;
extern const float frequencyScaler;

extern volatile uint32_t risingEdgeTime;
extern volatile uint32_t fallingEdgeTime;
extern volatile uint32_t period;
extern volatile uint32_t pulseWidthUs;
extern volatile bool newData;

bool calculateFrequency();
SensorState validateSignal(float freq, uint32_t pulseWidth);
void frequencyToEthanolContent(float measuredFrequency, float scaler);
void pulseWidthToFuelTemperature(uint32_t pulseWidthUs);

void onSensorEdge();

void initCAN();
void sendZeitronixCANMessage();
