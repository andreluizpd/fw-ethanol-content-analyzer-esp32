#include "main.h"
#include <NimBLEDevice.h>
#include <cstring>

static NimBLECharacteristic* ethanolChar = nullptr;
static NimBLECharacteristic* temperatureChar = nullptr;

float ethanol = 0.f;
float fuelTemperature = 0.f;

float frequency = 0.f;
const float frequencyScaler = ETHANOL_FREQUENCY_SCALER;

volatile uint32_t risingEdgeTime = 0;
volatile uint32_t fallingEdgeTime = 0;
volatile uint32_t period = 0;
volatile uint32_t pulseWidthUs = 0;
volatile bool newData = false;

volatile uint32_t lastValidReadingMs = 0;
bool sensorTimedOut = false;

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pinMode(ECA_INPUT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECA_INPUT), onSensorEdge, CHANGE);

  setupBLE();

  Serial.println("Ethanol Content Analyzer - ESP32-C3");
  Serial.println("Waiting for sensor data...");
}

void loop()
{
  checkSensorTimeout();

  if (newData && calculateFrequency()) {
    lastValidReadingMs = millis();
    frequencyToEthanolContent(frequency, frequencyScaler);

    uint32_t capturedPulseWidth = pulseWidthUs;
    pulseWidthToFuelTemperature(capturedPulseWidth);

    newData = false;
    updateBLE();

    Serial.print("Period (us): ");
    Serial.print(period);
    Serial.print("\tFrequency: ");
    Serial.print(frequency, 1);
    Serial.print(" Hz");
    Serial.print("\tEthanol: ");
    Serial.print(ethanol, 1);
    Serial.print("%");
    Serial.print("\tPulse Width: ");
    Serial.print(capturedPulseWidth);
    Serial.print(" us");
    Serial.print("\tFuel Temp: ");
    Serial.print(fuelTemperature, 1);
    Serial.println(" C");
  }
}

void IRAM_ATTR onSensorEdge()
{
  uint32_t now = micros();
  bool level = digitalRead(ECA_INPUT);

  if (level) {
    if (risingEdgeTime > 0) {
      period = now - risingEdgeTime;
      newData = true;
    }
    if (fallingEdgeTime > 0) {
      pulseWidthUs = now - fallingEdgeTime;
    }
    risingEdgeTime = now;
  } else {
    fallingEdgeTime = now;
  }
}

bool calculateFrequency()
{
  uint32_t capturedPeriod = period;
  if (capturedPeriod == 0) {
    return false;
  }

  float tempFrequency = 1000000.f / (float)capturedPeriod;
  if (tempFrequency < MIN_FREQUENCY || tempFrequency > MAX_FREQUENCY) {
    return false;
  }

  if (frequency == 0) {
    frequency = tempFrequency;
  } else {
    frequency = (1 - FREQUENCY_ALPHA) * frequency + FREQUENCY_ALPHA * tempFrequency;
  }

  return true;
}

void frequencyToEthanolContent(float measuredFrequency, float scaler)
{
  ethanol = (measuredFrequency - E0_FREQUENCY) / scaler;
  ethanol = max(0.0f, min(100.0f, ethanol));
}

void pulseWidthToFuelTemperature(uint32_t pulseWidth)
{
  if (pulseWidth < PULSE_WIDTH_MIN_US || pulseWidth > PULSE_WIDTH_MAX_US) {
    return;
  }

  float clampedPulseWidth = max((float)PULSE_WIDTH_LOW_US, min((float)PULSE_WIDTH_HIGH_US, (float)pulseWidth));
  float rawTemp = TEMP_MIN
    + (clampedPulseWidth - PULSE_WIDTH_LOW_US) * (TEMP_MAX - TEMP_MIN)
      / (float)(PULSE_WIDTH_HIGH_US - PULSE_WIDTH_LOW_US);

  static bool tempInitialized = false;
  if (!tempInitialized) {
    fuelTemperature = rawTemp;
    tempInitialized = true;
  } else {
    fuelTemperature = (1 - TEMPERATURE_ALPHA) * fuelTemperature + TEMPERATURE_ALPHA * rawTemp;
  }
}

void setupBLE()
{
  NimBLEDevice::init(BLE_DEVICE_NAME);

  NimBLEServer* server = NimBLEDevice::createServer();
  NimBLEService* service = server->createService(BLE_SERVICE_UUID);

  ethanolChar = service->createCharacteristic(
    BLE_ETHANOL_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  ethanolChar->createDescriptor("2901")->setValue("Ethanol Content (%)");

  temperatureChar = service->createCharacteristic(
    BLE_TEMPERATURE_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  temperatureChar->createDescriptor("2901")->setValue("Fuel Temperature (C)");

  service->start();
  NimBLEDevice::startAdvertising();

  Serial.println("BLE advertising as \"" BLE_DEVICE_NAME "\"");
}

void updateBLE()
{
  char buf[16];

  snprintf(buf, sizeof(buf), "%.1f%%", ethanol);
  ethanolChar->setValue(reinterpret_cast<const uint8_t*>(buf), strlen(buf));
  ethanolChar->notify();

  snprintf(buf, sizeof(buf), "%.1f C", fuelTemperature);
  temperatureChar->setValue(reinterpret_cast<const uint8_t*>(buf), strlen(buf));
  temperatureChar->notify();
}

void checkSensorTimeout()
{
  if (lastValidReadingMs == 0) {
    return;
  }

  if (millis() - lastValidReadingMs > SENSOR_TIMEOUT_MS) {
    if (!sensorTimedOut) {
      sensorTimedOut = true;
      ethanol = FALLBACK_ETHANOL;
      Serial.println("WARNING: Flex fuel sensor timed out! Using fallback values.");
    }
  } else {
    sensorTimedOut = false;
  }
}
