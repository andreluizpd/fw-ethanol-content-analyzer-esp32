#include "main.h"
#include <NimBLEDevice.h>

static NimBLECharacteristic* ethanolChar = nullptr;
static NimBLECharacteristic* temperatureChar = nullptr;

// State variables
float ethanol = 0.f;
float fuelTemperature = 0.f;

// Frequency readings
float frequency = 0.f;
const float frequencyScaler = ETHANOL_FREQUENCY_SCALER;

// ISR shared variables
volatile uint32_t risingEdgeTime = 0;
volatile uint32_t period = 0;
volatile uint32_t pulseWidthUs = 0;
volatile bool newData = false;

// Timeout tracking
volatile uint32_t lastValidReadingMs = 0;
bool sensorTimedOut = false;

void setup()
{
  Serial.begin(115200);

  // Setup GPIO interrupt for sensor input
  pinMode(ECA_INPUT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECA_INPUT), onSensorEdge, CHANGE);

  setupBLE();

  Serial.println("Ethanol Content Analyzer - ESP32-C3");
  Serial.println("Waiting for sensor data...");
}

void loop()
{
  checkSensorTimeout();

  // get ethanol content from sensor frequency
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

/**
 * GPIO interrupt handler - fires on both rising and falling edges
 * Measures period (rising-to-rising) and pulse width (rising-to-falling)
 * to get both frequency (ethanol %) and pulse width (fuel temperature)
 */
void IRAM_ATTR onSensorEdge() {
  uint32_t now = micros();
  bool level = digitalRead(ECA_INPUT);

  if (level) {
    // Rising edge: calculate period from last rising edge
    if (risingEdgeTime > 0) {
      period = now - risingEdgeTime;
      newData = true;
    }
    risingEdgeTime = now;
  } else {
    // Falling edge: measure pulse width (rising-to-falling) in microseconds
    if (risingEdgeTime > 0) {
      pulseWidthUs = now - risingEdgeTime;
    }
  }
}

/**
 * Calculates frequency using the period measured by GPIO interrupts
 * @return boolean - true if valid frequency
 */
bool calculateFrequency() {
  uint32_t capturedPeriod = period;
  if (capturedPeriod == 0)
    return false;

  float tempFrequency = 1000000.f / (float)capturedPeriod; // period is in microseconds

  if (tempFrequency < MIN_FREQUENCY || tempFrequency > MAX_FREQUENCY)
    return false;

  // if we haven't calculated frequency, use the current frequency.
  // Otherwise, run it through an exponential filter to smooth readings
  if (frequency == 0)
    frequency = tempFrequency;
  else
    frequency = (1 - FREQUENCY_ALPHA) * frequency + FREQUENCY_ALPHA * tempFrequency;

  return true;
}

/**
 * Converts a sensor frequency to ethanol percentage
 * Ethanol % = Frequency (in Hz) - 50.0.
 * A value of 180 Hz - 190 Hz indicates contaminated fuel.
 * @param frequency - Input frequency (1 / period)
 * @param scaler - Value by which interpolate frequencies between E0 and E100
 */
void frequencyToEthanolContent(float frequency, float scaler) {
  ethanol = (frequency - E0_FREQUENCY) / scaler;

  // bound ethanol content by a max of 0 to 100
  ethanol = max(0.0f, min(100.0f, ethanol));
}

/**
 * Converts the pulse width of the sensor signal to fuel temperature
 * The Continental flex fuel sensor encodes temperature in the pulse width:
 * 1000 µs = -40°C, 5000 µs = +125°C (linear interpolation)
 * @param pw - Pulse width in microseconds
 */
void pulseWidthToFuelTemperature(uint32_t pw) {
  // Validate pulse width range
  if (pw < PULSE_WIDTH_MIN_US || pw > PULSE_WIDTH_MAX_US)
    return;

  // Linear interpolation: 1000 µs = -40°C, 5000 µs = +125°C
  float clampedPw = max((float)PULSE_WIDTH_LOW_US, min((float)PULSE_WIDTH_HIGH_US, (float)pw));
  float rawTemp = TEMP_MIN + (clampedPw - PULSE_WIDTH_LOW_US) * (TEMP_MAX - TEMP_MIN)
                  / (float)(PULSE_WIDTH_HIGH_US - PULSE_WIDTH_LOW_US);

  // EMA filter for temperature (changes slowly)
  static bool tempInitialized = false;
  if (!tempInitialized) {
    fuelTemperature = rawTemp;
    tempInitialized = true;
  } else {
    fuelTemperature = (1 - TEMPERATURE_ALPHA) * fuelTemperature + TEMPERATURE_ALPHA * rawTemp;
  }
}

/**
 * Initializes BLE server, service, and characteristics with notify support
 */
void setupBLE() {
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

/**
 * Pushes current ethanol and temperature values to BLE characteristics
 * Values are sent as UTF-8 strings for easy reading in LightBlue
 */
void updateBLE() {
  char buf[16];

  snprintf(buf, sizeof(buf), "%.1f%%", ethanol);
  ethanolChar->setValue(buf);
  ethanolChar->notify();

  snprintf(buf, sizeof(buf), "%.1f C", fuelTemperature);
  temperatureChar->setValue(buf);
  temperatureChar->notify();
}

/**
 * Checks if the sensor has timed out (no valid reading for SENSOR_TIMEOUT_MS)
 * Sets sensorTimedOut flag and applies fallback ethanol value
 */
void checkSensorTimeout() {
  if (lastValidReadingMs == 0) return;

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
