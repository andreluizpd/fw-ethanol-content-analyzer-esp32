#pragma once

#include "Arduino.h"

// I/O
#define ECA_INPUT                 4             // Flex fuel sensor signal input (use voltage divider from 5V!)

// constants
#define FREQUENCY_ALPHA           0.01f
#define TEMPERATURE_ALPHA         0.005f        // EMA alpha for fuel temperature smoothing

#define MIN_FREQUENCY             45.f          // Hz — below this, sensor is disconnected
#define MAX_FREQUENCY             155.f         // Hz — above this, contamination or fault

#define E0_FREQUENCY              50.f          // Hz
#define E100_FREQUENCY            150.f         // Hz
#define ETHANOL_FREQUENCY_SCALER  ((E100_FREQUENCY - E0_FREQUENCY) / 100.0f)

// Temperature (pulse width encoding)
#define TEMP_MIN                  -40.0f        // °C
#define TEMP_MAX                  125.0f        // °C
#define PULSE_WIDTH_MIN_US        900            // µs — below this, fuel temp invalid
#define PULSE_WIDTH_MAX_US        5100           // µs — above this, fuel temp invalid
#define PULSE_WIDTH_LOW_US        1000           // µs — maps to TEMP_MIN
#define PULSE_WIDTH_HIGH_US       5000           // µs — maps to TEMP_MAX

// Timeout
#define SENSOR_TIMEOUT_MS         30000          // 30 seconds with no valid reading = sensor failed

// Fallback
#define FALLBACK_ETHANOL          30.0f          // Brazilian fuel default (~E27/E30)

// State variables
extern float ethanol;
extern float fuelTemperature;

// Frequency readings
extern float frequency;
extern const float frequencyScaler;

// ISR shared variables
extern volatile uint32_t risingEdgeTime;
extern volatile uint32_t period;
extern volatile uint32_t pulseWidthUs;
extern volatile bool newData;

// Timeout tracking
extern volatile uint32_t lastValidReadingMs;
extern bool sensorTimedOut;

// Function declarations
bool calculateFrequency();
void frequencyToEthanolContent(float frequency, float scaler);
void pulseWidthToFuelTemperature(uint32_t pulseWidthUs);
void checkSensorTimeout();

void IRAM_ATTR onSensorEdge();
