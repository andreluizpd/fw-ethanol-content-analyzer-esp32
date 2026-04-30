#include "main.h"
#include <cstring>

// State variables
float ethanol = 0.f;
float fuelTemperature = 0.f;
bool canReady = false;
uint32_t lastSensorUpdateMs = 0;
SensorState sensorState = SENSOR_INITIALIZING;
uint8_t stablePulseCount = 0;

// CAN timing
uint32_t lastCANSend = 0;

// Frequency readings
float frequency = 0.f;
const float frequencyScaler = ETHANOL_FREQUENCY_SCALER;

// ISR shared variables
volatile uint32_t risingEdgeTime = 0;
volatile uint32_t fallingEdgeTime = 0;
volatile uint32_t period = 0;
volatile uint32_t pulseWidthUs = 0;
volatile bool newData = false;

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pinMode(ECA_INPUT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECA_INPUT), onSensorEdge, CHANGE);

  initCAN();

  // Reset the MCU if the main loop ever stalls for too long.
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  esp_task_wdt_init(&wdtConfig);
  esp_task_wdt_add(NULL);

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
      Serial.println("FAULT: Sensor timeout - no data received");
    }
  }

  if (newData && calculateFrequency()) {
    uint32_t capturedPulseWidth = pulseWidthUs;
    lastSensorUpdateMs = millis();
    newData = false;

    SensorState validation = validateSignal(frequency, capturedPulseWidth);

    if (validation == SENSOR_OK) {
      frequencyToEthanolContent(frequency, frequencyScaler);
      pulseWidthToFuelTemperature(capturedPulseWidth);

      if (sensorState == SENSOR_INITIALIZING) {
        stablePulseCount++;
        if (stablePulseCount >= STABLE_PULSES_REQUIRED) {
          sensorState = SENSOR_OK;
          Serial.println("Sensor: Stabilized - CAN output enabled");
        }
      } else {
        sensorState = SENSOR_OK;
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
        case SENSOR_PULSE_INVALID:
          Serial.printf("FAULT: Invalid pulse width %lu us - sensor disconnected\n", capturedPulseWidth);
          break;
        default:
          break;
      }
    }

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
    Serial.print(" C");
    Serial.printf("\tState: %d\n", sensorState);
  }

  if (canReady && (now - lastCANSend >= ZEITRONIX_CAN_INTERVAL_MS)) {
    lastCANSend = now;

    if (sensorState != SENSOR_TIMEOUT && sensorState != SENSOR_INITIALIZING) {
      sendZeitronixCANMessage();
    }
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
  if (tempFrequency < 0 || tempFrequency > MAX_FREQUENCY) {
    return false;
  }

  if (frequency == 0) {
    frequency = tempFrequency;
  } else {
    frequency = (1 - FREQUENCY_ALPHA) * frequency + FREQUENCY_ALPHA * tempFrequency;
  }

  return true;
}

SensorState validateSignal(float freq, uint32_t pulseWidth)
{
  if (freq < FREQ_UNDERRANGE_LIMIT) {
    return SENSOR_UNDERRANGE;
  }

  if (freq > FREQ_OVERRANGE_LIMIT) {
    return SENSOR_CONTAMINATED;
  }

  if (pulseWidth < PULSE_WIDTH_MIN_US || pulseWidth > PULSE_WIDTH_MAX_US) {
    return SENSOR_PULSE_INVALID;
  }

  return SENSOR_OK;
}

void frequencyToEthanolContent(float measuredFrequency, float scaler)
{
  ethanol = (measuredFrequency - E0_FREQUENCY) / scaler;
  ethanol = max(0.0f, min(100.0f, ethanol));
}

void pulseWidthToFuelTemperature(uint32_t pulseWidth)
{
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
  msg.identifier = ZEITRONIX_CAN_ID;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 8;

  memset(msg.data, 0, sizeof(msg.data));

  msg.data[0] = (uint8_t)constrain((int)roundf(ethanol), 0, 100);

  int tempRaw = (int)roundf(fuelTemperature) + 40;
  msg.data[1] = (uint8_t)constrain(tempRaw, 0, 255);

  msg.data[7] = (sensorState == SENSOR_OK) ? ZEITRONIX_SENSOR_OK : ZEITRONIX_SENSOR_FAULT;

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(10));
  if (result != ESP_OK) {
    Serial.printf("CAN: TX failed (0x%X)\n", result);
  }
}
