#include "main.h"
#include <NimBLEDevice.h>
#include <cstring>

static NimBLECharacteristic* ethanolChar = nullptr;
static NimBLECharacteristic* temperatureChar = nullptr;

// State variables
float ethanol = SAFE_ETHANOL_DEFAULT;
float fuelTemperature = SAFE_TEMP_DEFAULT;
bool canReady = false;
uint32_t lastSensorUpdateMs = 0;
uint32_t lastSerialReadingMs = 0;
SensorState sensorState = SENSOR_INITIALIZING;
uint8_t stablePulseCount = 0;

// CAN timing
uint32_t lastCANSend = 0;

// Frequency and duty cycle readings
float frequency = 0.f, dutyCycle = 0.f;
const float frequencyScaler = ETHANOL_FREQUENCY_SCALER;

// ISR shared variables
volatile uint32_t risingEdgeTime = 0;
volatile uint32_t fallingEdgeTime = 0;
volatile uint32_t period = 0;
volatile float rawDutyCycle = 0;
volatile bool newData = false;

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pinMode(ECA_INPUT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECA_INPUT), onSensorEdge, CHANGE);

  setupBLE();
  initCAN();

  // Reset the MCU if the main loop ever stalls for too long.
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  esp_err_t wdtInitResult = esp_task_wdt_init(&wdtConfig);
  if (wdtInitResult == ESP_OK) {
    esp_task_wdt_add(NULL);
  } else if (wdtInitResult == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_add(NULL);
  }

  Serial.println("Ethanol Content Analyzer - ESP32-C3");
  Serial.println("Waiting for sensor data...");
}

void loop()
{
  esp_task_wdt_reset();

  uint32_t now = millis();
  if (lastSensorUpdateMs > 0 && (now - lastSensorUpdateMs >= SENSOR_TIMEOUT_MS)) {
    if (sensorState != SENSOR_TIMEOUT) {
      sensorState = SENSOR_TIMEOUT;
      stablePulseCount = 0;
      ethanol = SAFE_ETHANOL_DEFAULT;
      fuelTemperature = SAFE_TEMP_DEFAULT;
      Serial.println("FAULT: Sensor timeout - no data received");
    }
  }

  if (newData && calculateFrequency()) {
    lastSensorUpdateMs = millis();
    newData = false;

    SensorState validation = validateSignal(frequency, dutyCycle);

    if (validation == SENSOR_OK) {
      frequencyToEthanolContent(frequency, frequencyScaler);
      dutyCycleToFuelTemperature(dutyCycle);

      if (sensorState == SENSOR_INITIALIZING) {
        stablePulseCount++;
        if (stablePulseCount >= STABLE_PULSES_REQUIRED) {
          sensorState = SENSOR_OK;
          Serial.println("Sensor: Stabilized - CAN output enabled");
          printSensorReading();
        }
      } else {
        sensorState = SENSOR_OK;
      }

      if (lastSerialReadingMs == 0 || (now - lastSerialReadingMs >= SERIAL_READING_INTERVAL_MS)) {
        printSensorReading();
      }
    } else {
      sensorState = validation;
      ethanol = SAFE_ETHANOL_DEFAULT;
      fuelTemperature = SAFE_TEMP_DEFAULT;
      stablePulseCount = 0;

      switch (validation) {
        case SENSOR_UNDERRANGE:
          Serial.printf("FAULT: Under-range frequency %.1f Hz - sensor failure/wiring\n", frequency);
          break;
        case SENSOR_CONTAMINATED:
          Serial.printf("FAULT: Over-range frequency %.1f Hz - water contamination\n", frequency);
          break;
        case SENSOR_DUTY_INVALID:
          Serial.printf("FAULT: Invalid duty cycle %.1f%% - sensor disconnected\n", dutyCycle);
          break;
        default:
          break;
      }
    }

    updateBLE();

  }

  if (canReady && (now - lastCANSend >= ZEITRONIX_CAN_INTERVAL_MS)) {
    lastCANSend = now;
    sendZeitronixCANMessage();
  }
}

void IRAM_ATTR onSensorEdge()
{
  uint32_t now = micros();
  bool level = digitalRead(ECA_INPUT);

  if (level) {
    if (risingEdgeTime > 0) {
      period = now - risingEdgeTime;
      if (fallingEdgeTime > risingEdgeTime) {
        uint32_t highTime = fallingEdgeTime - risingEdgeTime;
        rawDutyCycle = (float)highTime / (float)period * 100.0f;
      }
      newData = true;
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
  if (tempFrequency < 0 || tempFrequency > MAX_FREQUENCY) {
    return false;
  }

  dutyCycle = rawDutyCycle;

  if (frequency == 0) {
    frequency = tempFrequency;
  } else {
    frequency = (1 - FREQUENCY_ALPHA) * frequency + FREQUENCY_ALPHA * tempFrequency;
  }

  return true;
}

SensorState validateSignal(float freq, float duty)
{
  if (freq < FREQ_UNDERRANGE_LIMIT) {
    return SENSOR_UNDERRANGE;
  }

  if (freq > FREQ_OVERRANGE_LIMIT) {
    return SENSOR_CONTAMINATED;
  }

  if (duty < DUTY_CYCLE_MIN || duty > DUTY_CYCLE_MAX) {
    return SENSOR_DUTY_INVALID;
  }

  return SENSOR_OK;
}

void frequencyToEthanolContent(float measuredFrequency, float scaler)
{
  ethanol = (measuredFrequency - E0_FREQUENCY) / scaler;
  ethanol = max(0.0f, min(ETHANOL_MAX_CAP, ethanol));
}

void dutyCycleToFuelTemperature(float dutyCycle)
{
  float clampedDuty = max(10.0f, min(90.0f, dutyCycle));
  fuelTemperature = TEMP_MIN + (clampedDuty - 10.0f) * (TEMP_MAX - TEMP_MIN) / 80.0f;
}

void initCAN()
{
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = ZEITRONIX_CAN_SPEED;
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("CAN: TWAI driver installed");
  } else {
    Serial.println("CAN: TWAI driver install FAILED");
    canReady = false;
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("CAN: TWAI started (500 Kbps)");
    canReady = true;
  } else {
    Serial.println("CAN: TWAI start FAILED");
    twai_driver_uninstall();
    canReady = false;
  }
}

void sendZeitronixCANMessage()
{
  twai_message_t msg;
  float ethanolToSend = ethanol;
  float temperatureToSend = fuelTemperature;

  if (sensorState != SENSOR_OK) {
    ethanolToSend = SAFE_ETHANOL_DEFAULT;
    temperatureToSend = SAFE_TEMP_DEFAULT;
  }

  msg.identifier = ZEITRONIX_CAN_ID;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 8;

  memset(msg.data, 0, sizeof(msg.data));

  msg.data[0] = (uint8_t)constrain((int)roundf(ethanolToSend), 0, (int)ETHANOL_MAX_CAP);

  int tempRaw = (int)roundf(temperatureToSend) + 40;
  msg.data[1] = (uint8_t)constrain(tempRaw, 0, 255);

  msg.data[7] = (sensorState == SENSOR_OK) ? ZEITRONIX_SENSOR_OK : ZEITRONIX_SENSOR_FAULT;

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(10));
  if (result != ESP_OK) {
    Serial.printf("CAN: TX failed (0x%X)\n", result);
  }
}

void printSensorReading()
{
  lastSerialReadingMs = millis();
  Serial.print("Reading: ");
  Serial.print(frequency, 1);
  Serial.print(" Hz, Ethanol ");
  Serial.print(ethanol, 1);
  Serial.print("%, Temp ");
  Serial.print(fuelTemperature, 1);
  Serial.print(" C, State ");
  Serial.println((int)sensorState);
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
  if (ethanolChar == nullptr || temperatureChar == nullptr) {
    return;
  }

  char buf[16];

  snprintf(buf, sizeof(buf), "%.1f%%", ethanol);
  ethanolChar->setValue(reinterpret_cast<const uint8_t*>(buf), strlen(buf));
  ethanolChar->notify();

  snprintf(buf, sizeof(buf), "%.1f C", fuelTemperature);
  temperatureChar->setValue(reinterpret_cast<const uint8_t*>(buf), strlen(buf));
  temperatureChar->notify();
}
