# Flex Fuel (E85) Sensor — Code Implementation Reference

This document describes the rusEFI flex fuel sensor implementation in detail, intended as a reference for porting the logic to another project.

**Source files covered:**
- `firmware/controllers/sensors/flex_sensor.h` — Sensor class and frequency-to-ethanol converter
- `firmware/controllers/sensors/flex_sensor.cpp` — Interrupt callback and signal processing
- `firmware/init/sensor/init_flex.cpp` — Hardware initialization and teardown
- `firmware/controllers/sensors/sensor_checker.cpp` — OBD error code mapping
- `firmware/controllers/algo/fuel/fuel_computer.cpp` — Downstream usage (stoichiometric ratio)
- `firmware/controllers/algo/fuel_math.cpp` — Downstream usage (cranking fuel)

---

## 1. Sensor Hardware Protocol

The Continental/GM flex fuel sensor is a two-wire digital sensor that outputs a **variable-frequency square wave**. A single signal line encodes **two values** simultaneously:

| Parameter        | Encoding              | Range                  |
|------------------|-----------------------|------------------------|
| Ethanol %        | Frequency of signal   | 50 Hz (0%) – 150 Hz (100%) |
| Fuel temperature | Pulse width (low time, project implementation) | 1000 µs (−40°C) – 5000 µs (+125°C) |

The sensor continuously outputs this signal whenever powered. No request/response protocol is needed.

### Signal Diagram

```
        ┌────┐           ┌──────┐         ┌────┐
        │    │           │      │         │    │
   ─────┘    └───────────┘      └─────────┘    └─────
        ↑    ↑           ↑      ↑         ↑
        R1   F1          R2     F2        R3

   Frequency = 1 / (R2 - R1)           → ethanol %
   Pulse width = R2 - F1               → fuel temperature
   R = rising edge, F = falling edge
```

### Project Note: Measured Sensor Polarity

On this project hardware, bench and in-car oscilloscope captures showed that fuel
temperature is represented by the **low pulse width** rather than the high pulse width.
The firmware in this repository therefore measures temperature as:

```
lowPulseWidthUs = nextRisingEdge - previousFallingEdge
```

Frequency is still measured from rising edge to rising edge.

---

## 2. Hardware Initialization

**File:** `firmware/init/sensor/init_flex.cpp`

### Setup

The sensor input pin is configured as an **external interrupt (EXTI)** triggering on **both edges** (rising and falling). This is the only hardware requirement — a single digital GPIO input with interrupt capability.

```
Pin mode:      Digital input
Interrupt:     Both edges (rising + falling)
Pull-up/down:  External (sensor has its own output driver)
```

### Initialization sequence

```
1. Enable EXTI on the configured GPIO pin, both edges, with callback function
2. If EXTI setup fails (pin invalid, resource conflict) → abort, sensor won't work
3. Store the active pin for later teardown
4. Register the sensor in the sensor framework
   - If a separate analog fuel temp sensor exists, skip registering
     the flex sensor's fuel temperature output to avoid conflicts
```

### Teardown

```
1. Unregister sensor from framework (ethanol % and fuel temp)
2. Disable EXTI on the pin
3. Clear stored pin reference
```

### Signal Inversion

The callback reads the current pin state and XORs it with a configurable `flexSensorInverted` flag. This handles sensors or circuits that invert the signal polarity:

```
actual_value = read_pin(flexPin) XOR config.flexSensorInverted
```

---

## 3. Signal Processing (Interrupt Callback)

**File:** `firmware/controllers/sensors/flex_sensor.cpp`, function `FlexSensor::callback`

The callback fires on **every edge** (both rising and falling). The `value` parameter indicates the current pin state after reading (true = high/rising, false = low/falling).

### State Machine

The callback implements a simple state machine with one state variable: `gotRising` (bool).

```
Initial state: gotRising = false

On RISING edge (value == true):
    if gotRising == false:
        // First rising edge ever — just start timing, can't compute frequency yet
        Reset frequency timer (flexFreq)
        Set gotRising = true
    else:
        // Second+ rising edge — compute frequency from period
        elapsed = flexFreq.getElapsedSecondsAndReset()
        frequency = 1.0 / elapsed
        Post raw frequency value to sensor framework (triggers conversion)

    // Always reset pulse-width timer on rising edge
    Reset pulse timer (flexPulse)

On FALLING edge (value == false):
    Record falling edge timestamp

On RISING edge (value == true):
    if there was a previous falling edge:
        // Measure low pulse width (time from falling edge to this rising edge)
        pulseWidthUs = now - lastFallingEdge
        Process fuel temperature from pulse width (see section 3.2)
```

### 3.1. Ethanol Percentage from Frequency

**File:** `firmware/controllers/sensors/flex_sensor.h`, class `FlexConverter`

When `postRawValue(frequency)` is called, the sensor framework invokes the `FlexConverter::convert()` function:

```
Input:  frequency (Hz), measured from rising-edge period

Step 1 — Range validation:
    if frequency < 45 Hz  → return Error::Low    (sensor disconnected or dead)
    if frequency > 155 Hz → return Error::High   (methanol contamination or fault)

Step 2 — Linear conversion:
    ethanol_pct = clamp(0, frequency - 50, 100)
    // 50 Hz → 0%, 150 Hz → 100%

Step 3 — Low-pass filter:
    ethanol_pct = biquad_lowpass(ethanol_pct)
    // Configured as: sample_rate=100 Hz, cutoff=1 Hz
    // Effective -3dB at 0.5–1.5 Hz depending on actual sensor frequency

Output: filtered ethanol percentage (0–100)
```

**Why filter?** The sensor frequency jitters slightly cycle-to-cycle. A biquad low-pass at ~1 Hz smooths this while still tracking real fuel changes (which happen on the timescale of minutes, not milliseconds).

### 3.2. Fuel Temperature from Pulse Width

Measured on rising edge, from the time elapsed since the last falling edge:

```
Input:  pulseWidthUs (microseconds)

Validation:
    if pulseWidthUs < 900  → invalidate fuel temp, emit warning (Error::Low)
    if pulseWidthUs > 5100 → invalidate fuel temp, emit warning (Error::High)

Conversion (linear interpolation, clamped):
    tempC = interpolate(1000 µs → -40°C, 5000 µs → +125°C, pulseWidthUs)

Filtering:
    tempC = biquad_lowpass(tempC)
    // Configured as: sample_rate=1 Hz, cutoff=0.01 Hz (very slow, temp changes slowly)

Output: filtered fuel temperature in °C
```

---

## 4. Timeout Handling

The `FlexSensor` is constructed with a **30-second timeout** (`MS2NT(30000)`).

This means: if no new valid reading is posted for 30 seconds, the sensor framework automatically marks the ethanol % value as **timed out / invalid**.

This long timeout is intentional — during engine cranking, the sensor may not produce a valid signal for several seconds. A short timeout would cause false "sensor failed" states during normal startup.

When downstream code reads the sensor after timeout:
- `Sensor::get(SensorType::FuelEthanolPercent)` returns an invalid result
- Consumers must handle this (e.g., `.value_or(50)` defaults to 50% ethanol)

---

## 5. Error Handling Summary

### 5.1. Signal-Level Errors (detected in converter/callback)

| Condition              | Error Code         | Meaning                                    |
|------------------------|--------------------|--------------------------------------------|
| Frequency < 45 Hz     | `UnexpectedCode::Low`  | No signal / sensor disconnected        |
| Frequency > 155 Hz    | `UnexpectedCode::High` | Methanol contamination or sensor fault |
| Pulse width < 900 µs  | `UnexpectedCode::Low`  | Fuel temp signal out of range (low)    |
| Pulse width > 5100 µs | `UnexpectedCode::High` | Fuel temp signal out of range (high)   |
| No edges for 30 sec   | Timeout                | Sensor not producing signal            |

### 5.2. OBD Diagnostic Codes (detected in sensor_checker.cpp)

The sensor framework maps these to OBD-style fault codes:

| Condition | OBD Code                      |
|-----------|-------------------------------|
| Timeout   | `OBD_FlexSensor_Timeout`      |
| Low       | `OBD_FlexSensor_Low`          |
| High      | `OBD_FlexSensor_High`         |

### 5.3. Downstream Fallback Behavior

When the flex sensor is unavailable or invalid, consumers apply fallbacks:

- **Stoichiometric ratio** (`fuel_computer.cpp`): If no flex sensor is registered at all, uses only the primary stoich ratio (gasoline). If sensor is registered but reading fails, the current code uses `flex.Value` without a fallback (has a TODO comment noting this gap).

- **Cranking fuel** (`fuel_math.cpp`): Uses `.value_or(50)` — defaults to **50% ethanol** if the sensor fails. This is a safe middle-ground that avoids both too-lean (pure E85 assumption) and too-rich (pure gasoline assumption) during cranking.

---

## 6. Downstream Usage of Sensor Data

### 6.1. Stoichiometric Ratio Adjustment

```
if no flex sensor registered:
    return stoichRatioPrimary               // e.g., 14.7 for gasoline

stoich = interpolate(
    0%   → stoichRatioPrimary,              // e.g., 14.7 (gasoline)
    100% → stoichRatioSecondary,            // e.g., 9.0  (ethanol)
    ethanol_pct
)
```

This directly affects the target air-fuel ratio for the entire fuel calculation.

### 6.2. Cranking Fuel Adjustment

When `flexCranking` is enabled:

```
e0_mult  = lookup(CLT, crankingFuelCoef)       // gasoline cranking multiplier
e85_mult = lookup(CLT, crankingFuelCoefE100)   // E100 cranking multiplier

cranking_multiplier = interpolate(
    0%  → e0_mult,
    85% → e85_mult,
    ethanol_pct                                 // defaults to 50% if sensor fails
)
```

Note: the interpolation range is 0–85%, not 0–100%. This means at 85%+ ethanol, the full E100 multiplier is used.

---

## 7. Porting Checklist

To reimplement this logic on another platform, you need:

### Hardware
- [ ] A digital GPIO input with **external interrupt on both edges**
- [ ] Sufficient interrupt latency (the sensor runs at 50–150 Hz, so timing precision of ~10 µs is adequate)
- [ ] A microsecond-resolution timer or tick counter accessible from interrupt context

### Core Algorithm
- [ ] **Rising edge handler**: measure period between consecutive rising edges → frequency → ethanol %
- [ ] **Falling edge handler**: measure pulse width from last rising edge → fuel temperature
- [ ] **First-edge guard**: discard the first rising edge (no previous edge to measure period from)
- [ ] **Frequency-to-ethanol conversion**: `ethanol_pct = clamp(0, frequency_hz - 50, 100)`
- [ ] **Pulse-width-to-temperature conversion**: linear interpolation from 1000 µs/−40°C to 5000 µs/+125°C

### Filtering
- [ ] **Biquad low-pass filter** on ethanol % (sample rate ≈ sensor frequency, cutoff ≈ 1 Hz)
- [ ] **Biquad low-pass filter** on fuel temperature (much lower cutoff, ~0.01 Hz)
- [ ] Filters must be safe to call from interrupt context (no dynamic allocation)

### Error Handling
- [ ] **Frequency range check**: reject < 45 Hz and > 155 Hz
- [ ] **Pulse width range check**: reject < 900 µs and > 5100 µs
- [ ] **Timeout**: if no valid reading for 30 seconds, mark sensor as failed
- [ ] **Fallback values**: decide on defaults when sensor fails (rusEFI uses 50% ethanol for cranking)
- [ ] **Signal inversion option**: XOR pin reading with a config flag for inverted signals

### Optional
- [ ] Diagnostic/debug output (callback counters, raw frequency, raw pulse width)
- [ ] OBD-II fault code integration
- [ ] Separate analog fuel temp sensor override (skip flex temp if a dedicated sensor exists)

---

## 8. Pseudocode Summary

```
// --- State ---
got_rising = false
freq_timer = Timer()
pulse_timer = Timer()
ethanol_filter = BiquadLowpass(sample_rate=100, cutoff=1.0)
temp_filter = BiquadLowpass(sample_rate=1, cutoff=0.01)
last_valid_time = 0
TIMEOUT = 30 seconds

// --- Interrupt handler (both edges) ---
function on_edge(now, pin_is_high):
    if pin_is_high:                          // Rising edge
        if got_rising:
            period = freq_timer.elapsed(now)
            freq_timer.reset(now)
            frequency = 1.0 / period

            if frequency < 45:
                report_error(LOW)
            else if frequency > 155:
                report_error(HIGH)
            else:
                ethanol_pct = clamp(0, frequency - 50, 100)
                ethanol_pct = ethanol_filter.process(ethanol_pct)
                publish_ethanol(ethanol_pct)
                last_valid_time = now
        else:
            freq_timer.reset(now)

        pulse_timer.reset(now)
        got_rising = true

    else:                                    // Falling edge
        if got_rising:
            pw_us = pulse_timer.elapsed_us(now)

            if pw_us < 900:
                invalidate_fuel_temp(LOW)
            else if pw_us > 5100:
                invalidate_fuel_temp(HIGH)
            else:
                temp_c = lerp(1000, -40, 5000, 125, pw_us)
                temp_c = temp_filter.process(temp_c)
                publish_fuel_temp(temp_c)

// --- Periodic check (e.g., main loop) ---
function check_timeout(now):
    if (now - last_valid_time) > TIMEOUT:
        invalidate_ethanol()
```
