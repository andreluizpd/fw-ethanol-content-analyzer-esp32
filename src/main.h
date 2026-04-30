#pragma once

#include "Arduino.h"

#define ECA_INPUT                 4

#define FREQUENCY_ALPHA           0.01f
#define TEMPERATURE_ALPHA         0.005f

#define MIN_FREQUENCY             45.f
#define MAX_FREQUENCY             155.f

#define E0_FREQUENCY              50.f
#define E100_FREQUENCY            150.f
#define ETHANOL_FREQUENCY_SCALER  ((E100_FREQUENCY - E0_FREQUENCY) / 100.0f)

#define TEMP_MIN                  -40.0f
#define TEMP_MAX                  125.0f
#define PULSE_WIDTH_MIN_US        900
#define PULSE_WIDTH_MAX_US        5100
#define PULSE_WIDTH_LOW_US        1000
#define PULSE_WIDTH_HIGH_US       5000

#define SENSOR_TIMEOUT_MS         30000
#define FALLBACK_ETHANOL          30.0f

#define BLE_DEVICE_NAME           "BimmerWest Flex Kit"
#define BLE_SERVICE_UUID          "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_ETHANOL_UUID          "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_TEMPERATURE_UUID      "beb5483f-36e1-4688-b7f5-ea07361b26a8"

extern float ethanol;
extern float fuelTemperature;

extern float frequency;
extern const float frequencyScaler;

extern volatile uint32_t risingEdgeTime;
extern volatile uint32_t fallingEdgeTime;
extern volatile uint32_t period;
extern volatile uint32_t pulseWidthUs;
extern volatile bool newData;

extern volatile uint32_t lastValidReadingMs;
extern bool sensorTimedOut;

bool calculateFrequency();
void frequencyToEthanolContent(float measuredFrequency, float scaler);
void pulseWidthToFuelTemperature(uint32_t pulseWidthUs);
void checkSensorTimeout();
void setupBLE();
void updateBLE();

void IRAM_ATTR onSensorEdge();
