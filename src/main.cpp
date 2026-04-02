#include "main.h"

// State variables
float ethanol = 0.f;
float fuelTemperature = 0.f;
bool canReady = false;
uint32_t lastSensorUpdateMs = 0;
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

  // Setup GPIO interrupt for sensor input
  pinMode(ECA_INPUT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECA_INPUT), onSensorEdge, CHANGE);

  // Initialize TWAI (CAN) peripheral
  initCAN();

  // Initialize software watchdog (auto-resets ESP32 if main loop freezes)
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  Serial.println("Ethanol Content Analyzer - ESP32-C3");
  Serial.println("Waiting for sensor data...");
}

void loop()
{
  // Feed the watchdog to prove the main loop is alive
  esp_task_wdt_reset();

  // Check for sensor timeout
  uint32_t now = millis();
  if (lastSensorUpdateMs > 0 && (now - lastSensorUpdateMs >= SENSOR_TIMEOUT_MS)) {
    if (sensorState != SENSOR_TIMEOUT) {
      sensorState = SENSOR_TIMEOUT;
      Serial.println("FAULT: Sensor timeout - no data received");
    }
  }

  // Process new sensor data
  if (newData && calculateFrequency()) {
    lastSensorUpdateMs = millis();
    newData = false;

    // Validate signal before using it
    SensorState validation = validateSignal(frequency, dutyCycle);

    if (validation == SENSOR_OK) {
      frequencyToEthanolContent(frequency, frequencyScaler);
      dutyCycleToFuelTemperature(dutyCycle);

      // Startup stabilization: count stable pulses before trusting data
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
      // Signal is invalid - apply safe defaults and log the fault
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

    Serial.print("Period (us): ");
    Serial.print(period);
    Serial.print("\tFrequency: ");
    Serial.print(frequency, 1);
    Serial.print(" Hz");
    Serial.print("\tEthanol: ");
    Serial.print(ethanol, 1);
    Serial.print("%");
    Serial.print("\tDuty Cycle: ");
    Serial.print(dutyCycle, 1);
    Serial.print("%");
    Serial.print("\tFuel Temp: ");
    Serial.print(fuelTemperature, 1);
    Serial.print(" C");
    Serial.printf("\tState: %d\n", sensorState);
  }

  // Send CAN message at Zeitronix ECA-2 update rate (4 Hz)
  // Only send if CAN is ready AND sensor has stabilized (not initializing or timed out)
  if (canReady && (now - lastCANSend >= ZEITRONIX_CAN_INTERVAL_MS)) {
    lastCANSend = now;

    if (sensorState == SENSOR_TIMEOUT || sensorState == SENSOR_INITIALIZING) {
      // Cease broadcast — don't send stale/unvalidated data to the DME
    } else {
      sendZeitronixCANMessage();
    }
  }
}

/**
 * GPIO interrupt handler - fires on both rising and falling edges
 * Measures period (rising-to-rising) and pulse width (rising-to-falling)
 * to get both frequency (ethanol %) and duty cycle (fuel temperature)
 */
void IRAM_ATTR onSensorEdge() {
  uint32_t now = micros();
  bool level = digitalRead(ECA_INPUT);

  if (level) {
    // Rising edge: calculate period from last rising edge
    if (risingEdgeTime > 0) {
      period = now - risingEdgeTime;

      // Calculate duty cycle from the previous pulse width
      if (fallingEdgeTime > risingEdgeTime) {
        uint32_t highTime = fallingEdgeTime - risingEdgeTime;
        rawDutyCycle = (float)highTime / (float)period * 100.0f;
      }

      newData = true;
    }
    risingEdgeTime = now;
  } else {
    // Falling edge: record time for pulse width calculation
    fallingEdgeTime = now;
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

  if (tempFrequency < 0 || tempFrequency > MAX_FREQUENCY)
    return false;

  // Capture duty cycle from ISR
  dutyCycle = rawDutyCycle;

  // if we haven't calculated frequency, use the current frequency.
  // Otherwise, run it through an exponential filter to smooth readings
  if (frequency == 0)
    frequency = tempFrequency;
  else
    frequency = (1 - FREQUENCY_ALPHA) * frequency + FREQUENCY_ALPHA * tempFrequency;

  return true;
}

/**
 * Validates the sensor signal against expected ranges.
 * @param freq - Current frequency reading
 * @param duty - Current duty cycle reading
 * @return SensorState indicating the validation result
 */
SensorState validateSignal(float freq, float duty) {
  if (freq < FREQ_UNDERRANGE_LIMIT)
    return SENSOR_UNDERRANGE;

  if (freq > FREQ_OVERRANGE_LIMIT)
    return SENSOR_CONTAMINATED;

  if (duty < DUTY_CYCLE_MIN || duty > DUTY_CYCLE_MAX)
    return SENSOR_DUTY_INVALID;

  return SENSOR_OK;
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
 * Converts the duty cycle of the sensor signal to fuel temperature
 * The Continental flex fuel sensor encodes temperature in the pulse width:
 * Duty cycle maps linearly from -40°C to 125°C across the 10%-90% range
 * @param dutyCycle - Duty cycle percentage (0-100)
 */
void dutyCycleToFuelTemperature(float dutyCycle) {
  // Clamp duty cycle to valid range
  float clampedDuty = max(10.0f, min(90.0f, dutyCycle));

  // Linear interpolation: 10% duty = -40°C, 90% duty = 125°C
  fuelTemperature = TEMP_MIN + (clampedDuty - 10.0f) * (TEMP_MAX - TEMP_MIN) / 80.0f;
}

/**
 * Initializes the ESP32-C3 TWAI peripheral for CAN bus output
 * Configured to match Zeitronix ECA-2 CAN Bus defaults (500 Kbps, normal mode)
 */
void initCAN() {
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

/**
 * Sends a CAN message matching the Zeitronix ECA-2 CAN Bus protocol:
 *   Data[0] = Ethanol % (0-100)
 *   Data[1] = Fuel Temperature in C + 40 offset (raw byte)
 *   Data[7] = Sensor status (0x00 = OK, 0x01 = fault)
 */
void sendZeitronixCANMessage() {
  twai_message_t msg;
  msg.identifier = ZEITRONIX_CAN_ID;
  msg.extd = 0;           // Standard 11-bit ID
  msg.rtr = 0;
  msg.data_length_code = 8;

  memset(msg.data, 0, 8);

  // Data[0]: Ethanol percentage (0-100), clamped to uint8
  msg.data[0] = (uint8_t)constrain((int)roundf(ethanol), 0, 100);

  // Data[1]: Fuel temperature with +40 offset (raw = temp_C + 40)
  int tempRaw = (int)roundf(fuelTemperature) + 40;
  msg.data[1] = (uint8_t)constrain(tempRaw, 0, 255);

  // Data[7]: Sensor status from state machine
  msg.data[7] = (sensorState == SENSOR_OK) ? ZEITRONIX_SENSOR_OK : ZEITRONIX_SENSOR_FAULT;

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(10));
  if (result != ESP_OK) {
    Serial.printf("CAN: TX failed (0x%X)\n", result);
  }
}
