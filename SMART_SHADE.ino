/*
  Window-mounted real-time radiant heat sensing and variable smart shade
  Target board: Arduino Nano ESP32
  REQUIRED: Tools > Pin Numbering > By GPIO number (legacy)
  Reason: OneWire 2.3.8 bypasses the Nano ESP32 Arduino-pin remapping.

  IMPORTANT ELECTRICAL RULES
  - Every Nano ESP32 GPIO/ADC signal must stay between 0 V and 3.3 V.
  - Power the 28BYJ-48/ULN2003 from a separate regulated 5 V supply.
  - Connect the external 5 V GND and Nano ESP32 GND together.
  - Only the Upper Endstop is used. D5 is intentionally unused.
  - The lower travel limit is a stored software step limit with an 8% safety margin.
  - This firmware starts in MANUAL after boot homing unless AUTO_START_ENABLED is true.
*/

#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <Adafruit_MLX90614.h>
#include <AccelStepper.h>
#include <Preferences.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// =====================================================
// USER CONFIGURATION
// Change experimental values here, then upload again.
// =====================================================
namespace UserConfig {

// Nano header-pin aliases. Keep these as Dn/An aliases, not plain numbers.
constexpr uint8_t PIN_UV = A0;
constexpr uint8_t PIN_LIGHT = A1;
// Final wiring: physical Nano ESP32 header pin D2.
constexpr uint8_t PIN_DS18B20 = D2;
constexpr uint8_t PIN_DHT11 = D3;
constexpr uint8_t PIN_UPPER_ENDSTOP = D4;
constexpr uint8_t PIN_MOTOR_IN1 = D6;
constexpr uint8_t PIN_MOTOR_IN2 = D7;
constexpr uint8_t PIN_MOTOR_IN3 = D8;
constexpr uint8_t PIN_MOTOR_IN4 = D9;
constexpr uint8_t MLX90614_I2C_ADDRESS = 0x5A;

constexpr bool MOTOR_DIRECTION_INVERT = false;
constexpr bool ENDSTOP_ACTIVE_LOW = true;
constexpr bool AUTO_START_ENABLED = false;
constexpr bool HOLD_MOTOR_WHEN_IDLE = false;
constexpr bool EEI_CONTINUOUS_POSITION = false;
constexpr bool PERIODIC_STATUS_ENABLED_AT_BOOT = false;

// AccelStepper HALF4WIRE half-steps per second and half-steps/s^2.
// TEST VALUE - MUST BE VERIFIED WITH THE ASSEMBLED MECHANISM.
constexpr float HOMING_SPEED = 180.0f;
constexpr float HOMING_ACCELERATION = 100.0f;
constexpr float CALIBRATION_SPEED = 220.0f;
constexpr float CALIBRATION_ACCELERATION = 120.0f;
constexpr float NORMAL_MOVE_SPEED = 400.0f;
constexpr float NORMAL_MOVE_ACCELERATION = 220.0f;

constexpr uint32_t HOMING_TIMEOUT_MS = 90000UL;
constexpr uint32_t MOVE_TIMEOUT_MS = 60000UL;
// The previous assembled test was about 5,766 steps. These conservative
// ceilings still allow more than twice that travel while preventing runaway.
constexpr long HOMING_MAX_STEPS = 14000L;
constexpr long CALIBRATION_MEASURE_MAX_STEPS = 12000L;
constexpr long MAX_MEASUREMENT_JOG_STEPS = 2000L;
constexpr long MIN_VALID_CALIBRATION_STEPS = 300L;
constexpr long POSITION_ERROR_TOLERANCE_STEPS = 100L;
// CAL SET stores only 92% of the measured physical deployment length.
// This leaves an 8% software margin before the rail's physical bottom stop.
constexpr float DEPLOY_SAFETY_FACTOR = 0.92f;

constexpr uint32_t ENDSTOP_DEBOUNCE_MS = 20UL;
constexpr uint32_t FAST_SENSOR_INTERVAL_MS = 500UL;
constexpr uint32_t DS18B20_REQUEST_INTERVAL_MS = 1000UL;
constexpr uint32_t DS18B20_CONVERSION_MS = 750UL;
constexpr uint32_t DHT_INTERVAL_MS = 2500UL;
constexpr uint32_t MLX_RETRY_INTERVAL_MS = 5000UL;
constexpr uint32_t SENSOR_STALE_MS = 12000UL;
constexpr uint16_t SENSOR_WARNING_FAILURE_COUNT = 3;
constexpr uint16_t ANALOG_RAIL_WARNING_COUNT = 20;

constexpr uint32_t EEI_INTERVAL_MS = 2000UL;
constexpr uint32_t AUTO_CONTROL_INTERVAL_MS = 1000UL;
constexpr uint32_t CONTROL_STABLE_TIME_MS = 10000UL;
constexpr uint32_t AUTO_SENSOR_GRACE_MS = 10000UL;
constexpr uint32_t NIGHT_CONFIRM_TIME_MS = 30000UL;
constexpr float MIN_POSITION_CHANGE_PERCENT = 10.0f;
constexpr float EEI_HYSTERESIS = 0.03f;

constexpr uint32_t STATUS_INTERVAL_MS = 5000UL;
constexpr uint32_t LOG_INTERVAL_MS = 2000UL;
constexpr uint32_t SERIAL_BAUD = 115200UL;

constexpr uint8_t ADC_BITS = 12;
constexpr uint16_t ADC_MAX = 4095;
constexpr float ADC_VOLTAGE = 3.3f;  // Approximate voltage conversion only.
constexpr float ANALOG_EMA_ALPHA = 0.20f;
constexpr float TEMPERATURE_EMA_ALPHA = 0.18f;
constexpr float HUMIDITY_EMA_ALPHA = 0.20f;

// TEST VALUE - MUST BE CALIBRATED USING REAL SENSOR DATA.
// BRIGHT may be lower than DARK for an inverse-output light module.
constexpr float UV_MIN = 0.0f;
constexpr float UV_MAX = 1600.0f;
constexpr float LIGHT_MIN = 100.0f;
constexpr float LIGHT_MAX = 3500.0f;
constexpr float OUTDOOR_TEMP_MIN = 20.0f;
constexpr float OUTDOOR_TEMP_MAX = 40.0f;
constexpr float INDOOR_TEMP_MIN = 22.0f;
constexpr float INDOOR_TEMP_MAX = 35.0f;
constexpr float WINDOW_TEMP_MIN = 22.0f;
constexpr float WINDOW_TEMP_MAX = 55.0f;
constexpr float WINDOW_DELTA_MIN = 1.0f;
constexpr float WINDOW_DELTA_MAX = 15.0f;
constexpr float HUMIDITY_MIN = 40.0f;
constexpr float HUMIDITY_MAX = 80.0f;

// TEST VALUE - MUST BE CALIBRATED USING REAL SENSOR DATA.
// Initial weights sum to 1.00. Missing non-critical data is re-normalized.
constexpr float W_UV = 0.18f;
constexpr float W_LIGHT = 0.18f;
constexpr float W_OUT_TEMP = 0.10f;
constexpr float W_IN_TEMP = 0.20f;
constexpr float W_WINDOW = 0.27f;
constexpr float W_HUMIDITY = 0.07f;

// TEST VALUE - MUST BE CALIBRATED USING REAL SENSOR DATA.
constexpr float EEI_LEVEL_1 = 0.20f;
constexpr float EEI_LEVEL_2 = 0.40f;
constexpr float EEI_LEVEL_3 = 0.60f;
constexpr float EEI_LEVEL_4 = 0.80f;
constexpr float NIGHT_UV_MAX = 0.05f;
constexpr float NIGHT_LIGHT_MAX = 0.05f;

// Version 3 invalidates the former two-Endstop calibration automatically.
constexpr uint32_t CALIBRATION_STORAGE_VERSION = 3UL;
constexpr char PREFERENCES_NAMESPACE[] = "smartshade";
}  // namespace UserConfig

using namespace UserConfig;

enum class SystemState : uint8_t {
  STARTUP,
  HOMING,
  IDLE,
  MOVING_UP,
  MOVING_DOWN,
  AUTO_IDLE,
  ERROR,
  EMERGENCY_STOP
};

enum class ControlMode : uint8_t { MANUAL, AUTO };

enum class ErrorCode : uint8_t {
  NONE,
  HOMING_TIMEOUT,
  MOVE_TIMEOUT,
  POSITION_INVALID,
  CALIBRATION_INVALID,
  EMERGENCY_STOP_REQUESTED
};

enum class HomeReason : uint8_t {
  BOOT,
  COMMAND,
  CALIBRATION_START,
  CALIBRATION_RETURN,
  AUTO_POSITION_ZERO
};

struct SensorHealth {
  bool hasGoodValue;
  bool valid;
  bool warningActive;
  uint32_t lastGoodMs;
  uint16_t consecutiveFailures;
  uint32_t totalFailures;
};

struct AnalogReading {
  uint16_t raw;
  float filteredRaw;
  float voltage;
  float normalized;
  bool initialized;
  bool valid;
  uint16_t railCount;
};

struct ScalarReading {
  float value;
  SensorHealth health;
};

struct SensorData {
  AnalogReading uv;
  AnalogReading light;
  ScalarReading outdoorTemp;
  ScalarReading indoorTemp;
  ScalarReading indoorHumidity;
  ScalarReading windowTemp;
  ScalarReading mlxAmbientTemp;
  float outdoorNorm;
  float indoorNorm;
  float humidityNorm;
  float windowAbsoluteNorm;
  float windowDeltaNorm;
  float windowControlNorm;
  float eei;
  bool eeiValid;
  bool night;
  uint32_t lastEeiMs;
};

struct DebouncedEndstop {
  uint8_t pin;
  bool rawPressed;
  bool stablePressed;
  uint32_t rawChangedMs;
};

OneWire oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);
DHT dht(PIN_DHT11, DHT11);
Adafruit_MLX90614 mlx90614;
Preferences preferences;

// The 1,3,2,4 constructor order is intentional for 28BYJ-48 + ULN2003.
// Physical wiring remains D6->IN1, D7->IN2, D8->IN3, D9->IN4.
AccelStepper stepper(AccelStepper::HALF4WIRE,
                     PIN_MOTOR_IN1, PIN_MOTOR_IN3,
                     PIN_MOTOR_IN2, PIN_MOTOR_IN4);

SensorData sensors{};
DebouncedEndstop upperEndstop{};

SystemState systemState = SystemState::STARTUP;
ControlMode controlMode = ControlMode::MANUAL;
ErrorCode errorCode = ErrorCode::NONE;
HomeReason homeReason = HomeReason::BOOT;

bool mlxReady = false;
bool preferencesReady = false;
bool calibrationValid = false;
bool positionKnown = false;
bool dsConversionPending = false;
bool csvLoggingEnabled = false;
bool periodicStatusEnabled = PERIODIC_STATUS_ENABLED_AT_BOOT;
bool measurementMode = false;
bool movementStartedByAuto = false;
bool nightCandidateActive = false;
bool autoCandidateActive = false;
bool autoInvalidTimerActive = false;

long maxTravelSteps = 0;
long operationStartMotorPosition = 0;
long activeTargetLogicalSteps = 0;
float targetPercent = 0.0f;
float acceptedAutoTargetPercent = 0.0f;
float pendingAutoTargetPercent = 0.0f;

uint32_t operationStartMs = 0;
uint32_t lastFastSensorMs = 0;
uint32_t lastDhtMs = 0;
uint32_t dsRequestMs = 0;
uint32_t lastMlxRetryMs = 0;
uint32_t lastAutoControlMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastLogMs = 0;
uint32_t nightCandidateSinceMs = 0;
uint32_t autoCandidateSinceMs = 0;
uint32_t autoInvalidSinceMs = 0;

char serialBuffer[96];
uint8_t serialLength = 0;

// Forward declarations keep Arduino's sketch preprocessor from guessing custom types.
bool elapsedMs(uint32_t nowMs, uint32_t sinceMs, uint32_t periodMs);
float clamp01(float value);
float normalizeValue(float value, float minimum, float maximum);
uint16_t medianAnalogRead(uint8_t pin);
bool rawEndstopPressed(uint8_t pin);
void beginEndstop(DebouncedEndstop& sw, uint8_t pin, uint32_t nowMs);
void updateEndstop(DebouncedEndstop& sw, uint32_t nowMs);
void updateEndstops(uint32_t nowMs);
void initializeScalar(ScalarReading& reading);
void sensorSuccess(ScalarReading& reading, float value, float alpha,
                   uint32_t nowMs, const char* name);
void sensorFailure(ScalarReading& reading, const char* name);
void refreshSensorValidity(uint32_t nowMs);
void updateAnalogReading(AnalogReading& reading, uint8_t pin,
                         float minimum, float maximum, const char* name);
void initializeSensors(uint32_t nowMs);
void readFastSensors(uint32_t nowMs);
void updateDs18b20(uint32_t nowMs);
void readDht11(uint32_t nowMs);
void updateSensors(uint32_t nowMs);
void calculateNormalizedValues();
void calculateEEI(uint32_t nowMs);
bool automaticSensorsReady();
long logicalToMotor(long logicalSteps);
long motorToLogical(long motorSteps);
long currentLogicalSteps();
float currentPositionPercent();
void configureStepper(float maxSpeed, float acceleration);
void haltMotor(bool releaseOutputs);
void startHoming(HomeReason reason, uint32_t nowMs);
void completeHoming(uint32_t nowMs);
void startCalibration(uint32_t nowMs);
bool startMeasurementJog(long deltaSteps, uint32_t nowMs);
void saveMeasuredDeployment(uint32_t nowMs);
bool moveBlindToPercent(float percent, bool fromAuto, uint32_t nowMs);
void startManualDown(uint32_t nowMs);
void finishNormalMove(uint32_t nowMs);
void updateMotionState(uint32_t nowMs);
void enterError(ErrorCode code, const char* message);
void emergencyStop(const char* reason);
void loadCalibration();
void saveCalibration();
void clearCalibration();
float calculateTargetPosition(float eei, float referencePercent);
void updateAutomaticControl(uint32_t nowMs);
const char* stateName(SystemState state);
const char* modeName(ControlMode mode);
const char* errorName(ErrorCode code);
void printFloatOrNA(float value, bool valid, uint8_t decimals);
void printSensors();
void printStatus();
void printCsvHeader();
void printCsvValue(float value, bool valid, uint8_t decimals);
void printCsvRow(uint32_t nowMs);
void printHelp();
void processCommand(char* command, uint32_t nowMs);
void processSerialInput(uint32_t nowMs);

// =====================================================
// UTILITY, ENDSTOP, AND SENSOR IMPLEMENTATION
// =====================================================

bool elapsedMs(uint32_t nowMs, uint32_t sinceMs, uint32_t periodMs) {
  return static_cast<uint32_t>(nowMs - sinceMs) >= periodMs;
}

float clamp01(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

float normalizeValue(float value, float minimum, float maximum) {
  const float span = maximum - minimum;
  if (!isfinite(value) || fabsf(span) < 0.0001f) return 0.0f;
  return clamp01((value - minimum) / span);
}

uint16_t medianAnalogRead(uint8_t pin) {
  uint16_t samples[9];
  for (uint8_t i = 0; i < 9; ++i) {
    samples[i] = static_cast<uint16_t>(analogRead(pin));
  }
  for (uint8_t i = 1; i < 9; ++i) {
    const uint16_t key = samples[i];
    int8_t j = static_cast<int8_t>(i) - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      --j;
    }
    samples[j + 1] = key;
  }
  return samples[4];
}

bool rawEndstopPressed(uint8_t pin) {
  const bool levelHigh = digitalRead(pin) == HIGH;
  return ENDSTOP_ACTIVE_LOW ? !levelHigh : levelHigh;
}

void beginEndstop(DebouncedEndstop& sw, uint8_t pin, uint32_t nowMs) {
  sw.pin = pin;
  pinMode(pin, ENDSTOP_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
  sw.rawPressed = rawEndstopPressed(pin);
  sw.stablePressed = sw.rawPressed;
  sw.rawChangedMs = nowMs;
}

void updateEndstop(DebouncedEndstop& sw, uint32_t nowMs) {
  const bool pressed = rawEndstopPressed(sw.pin);
  if (pressed != sw.rawPressed) {
    sw.rawPressed = pressed;
    sw.rawChangedMs = nowMs;
  }
  if (sw.stablePressed != sw.rawPressed &&
      elapsedMs(nowMs, sw.rawChangedMs, ENDSTOP_DEBOUNCE_MS)) {
    sw.stablePressed = sw.rawPressed;
  }
}

void updateEndstops(uint32_t nowMs) {
  updateEndstop(upperEndstop, nowMs);
}

void initializeScalar(ScalarReading& reading) {
  reading.value = NAN;
  reading.health.hasGoodValue = false;
  reading.health.valid = false;
  reading.health.warningActive = false;
  reading.health.lastGoodMs = 0;
  reading.health.consecutiveFailures = 0;
  reading.health.totalFailures = 0;
}

void sensorSuccess(ScalarReading& reading, float value, float alpha,
                   uint32_t nowMs, const char* name) {
  if (!reading.health.hasGoodValue || !isfinite(reading.value)) {
    reading.value = value;
  } else {
    reading.value = alpha * value + (1.0f - alpha) * reading.value;
  }
  if (reading.health.warningActive) {
    Serial.print(F("SENSOR RECOVERED: "));
    Serial.println(name);
  }
  reading.health.hasGoodValue = true;
  reading.health.valid = true;
  reading.health.warningActive = false;
  reading.health.lastGoodMs = nowMs;
  reading.health.consecutiveFailures = 0;
}

void sensorFailure(ScalarReading& reading, const char* name) {
  if (reading.health.consecutiveFailures < 65535U) {
    ++reading.health.consecutiveFailures;
  }
  ++reading.health.totalFailures;
  if (!reading.health.warningActive &&
      reading.health.consecutiveFailures >= SENSOR_WARNING_FAILURE_COUNT) {
    reading.health.warningActive = true;
    Serial.print(F("SENSOR WARNING: "));
    Serial.print(name);
    Serial.print(F(" failed "));
    Serial.print(reading.health.consecutiveFailures);
    Serial.println(F(" consecutive reads; retaining the last good value temporarily."));
  }
}

void refreshSensorValidity(uint32_t nowMs) {
  ScalarReading* readings[] = {
      &sensors.outdoorTemp, &sensors.indoorTemp, &sensors.indoorHumidity,
      &sensors.windowTemp, &sensors.mlxAmbientTemp};
  for (ScalarReading* reading : readings) {
    reading->health.valid = reading->health.hasGoodValue &&
        static_cast<uint32_t>(nowMs - reading->health.lastGoodMs) <= SENSOR_STALE_MS;
  }
}

void updateAnalogReading(AnalogReading& reading, uint8_t pin,
                         float minimum, float maximum, const char* name) {
  reading.raw = medianAnalogRead(pin);
  if (!reading.initialized) {
    reading.filteredRaw = static_cast<float>(reading.raw);
    reading.initialized = true;
  } else {
    reading.filteredRaw = ANALOG_EMA_ALPHA * static_cast<float>(reading.raw)
                        + (1.0f - ANALOG_EMA_ALPHA) * reading.filteredRaw;
  }
  reading.voltage = reading.filteredRaw * ADC_VOLTAGE / static_cast<float>(ADC_MAX);
  reading.normalized = normalizeValue(reading.filteredRaw, minimum, maximum);
  reading.valid = reading.raw <= ADC_MAX;

  if (reading.raw == 0 || reading.raw >= ADC_MAX) {
    if (reading.railCount < 65535U) ++reading.railCount;
    if (reading.railCount == ANALOG_RAIL_WARNING_COUNT) {
      Serial.print(F("ANALOG WARNING: "));
      Serial.print(name);
      Serial.println(F(" is repeatedly at an ADC rail. Check wiring and module output voltage."));
    }
  } else {
    reading.railCount = 0;
  }
}

void initializeSensors(uint32_t nowMs) {
  sensors.uv = AnalogReading{};
  sensors.light = AnalogReading{};
  initializeScalar(sensors.outdoorTemp);
  initializeScalar(sensors.indoorTemp);
  initializeScalar(sensors.indoorHumidity);
  initializeScalar(sensors.windowTemp);
  initializeScalar(sensors.mlxAmbientTemp);
  sensors.eei = 0.0f;
  sensors.eeiValid = false;
  sensors.night = false;

  analogReadResolution(ADC_BITS);
  pinMode(PIN_UV, INPUT);
  pinMode(PIN_LIGHT, INPUT);

  Serial.println(F("[3] GUVA ADC channel initialized (module presence cannot be detected digitally)."));
  Serial.println(F("[4] Light ADC channel initialized (AO only; DO is unused)."));

  ds18b20.begin();
  ds18b20.setResolution(12);
  ds18b20.setWaitForConversion(false);
  Serial.print(F("[5] DS18B20 devices found: "));
  Serial.println(ds18b20.getDeviceCount());
  ds18b20.requestTemperatures();
  dsConversionPending = true;
  dsRequestMs = nowMs;

  dht.begin();
  Serial.println(F("[6] DHT11 initialized; first valid sample may take several seconds."));

  mlxReady = mlx90614.begin(MLX90614_I2C_ADDRESS, &Wire);
  Serial.print(F("[7] MLX90614 communication: "));
  Serial.println(mlxReady ? F("OK") : F("FAILED - will retry"));
  lastMlxRetryMs = nowMs;

  lastFastSensorMs = nowMs - FAST_SENSOR_INTERVAL_MS;
  lastDhtMs = nowMs - DHT_INTERVAL_MS;
}

void readFastSensors(uint32_t nowMs) {
  updateAnalogReading(sensors.uv, PIN_UV, UV_MIN, UV_MAX, "GUVA");
  updateAnalogReading(sensors.light, PIN_LIGHT, LIGHT_MIN, LIGHT_MAX, "LIGHT");

  if (!mlxReady) {
    if (elapsedMs(nowMs, lastMlxRetryMs, MLX_RETRY_INTERVAL_MS)) {
      lastMlxRetryMs = nowMs;
      mlxReady = mlx90614.begin(MLX90614_I2C_ADDRESS, &Wire);
      if (mlxReady) Serial.println(F("SENSOR RECOVERED: MLX90614 I2C"));
      else {
        sensorFailure(sensors.windowTemp, "MLX90614 object");
        sensorFailure(sensors.mlxAmbientTemp, "MLX90614 ambient");
      }
    }
    return;
  }

  const float objectC = mlx90614.readObjectTempC();
  const float ambientC = mlx90614.readAmbientTempC();
  if (isfinite(objectC) && objectC >= -20.0f && objectC <= 150.0f) {
    sensorSuccess(sensors.windowTemp, objectC, TEMPERATURE_EMA_ALPHA,
                  nowMs, "MLX90614 object");
  } else {
    sensorFailure(sensors.windowTemp, "MLX90614 object");
  }
  if (isfinite(ambientC) && ambientC >= -20.0f && ambientC <= 85.0f) {
    sensorSuccess(sensors.mlxAmbientTemp, ambientC, TEMPERATURE_EMA_ALPHA,
                  nowMs, "MLX90614 ambient");
  } else {
    sensorFailure(sensors.mlxAmbientTemp, "MLX90614 ambient");
  }
  if (sensors.windowTemp.health.consecutiveFailures >= 5U &&
      sensors.mlxAmbientTemp.health.consecutiveFailures >= 5U) {
    mlxReady = false;
    lastMlxRetryMs = nowMs;
  }
}

void updateDs18b20(uint32_t nowMs) {
  if (dsConversionPending && elapsedMs(nowMs, dsRequestMs, DS18B20_CONVERSION_MS)) {
    const float temperatureC = ds18b20.getTempCByIndex(0);
    const bool powerOnValue = fabsf(temperatureC - 85.0f) < 0.02f;
    const bool disconnected = temperatureC == DEVICE_DISCONNECTED_C ||
                              fabsf(temperatureC + 127.0f) < 0.02f;
    if (isfinite(temperatureC) && temperatureC >= -20.0f && temperatureC <= 80.0f &&
        !powerOnValue && !disconnected) {
      sensorSuccess(sensors.outdoorTemp, temperatureC, TEMPERATURE_EMA_ALPHA,
                    nowMs, "DS18B20");
    } else {
      sensorFailure(sensors.outdoorTemp, "DS18B20");
    }
    dsConversionPending = false;
  }

  if (!dsConversionPending &&
      elapsedMs(nowMs, dsRequestMs, DS18B20_REQUEST_INTERVAL_MS)) {
    ds18b20.requestTemperatures();
    dsRequestMs = nowMs;
    dsConversionPending = true;
  }
}

void readDht11(uint32_t nowMs) {
  const float humidity = dht.readHumidity();
  const float temperatureC = dht.readTemperature();
  if (isfinite(temperatureC) && temperatureC >= 0.0f && temperatureC <= 50.0f) {
    sensorSuccess(sensors.indoorTemp, temperatureC, TEMPERATURE_EMA_ALPHA,
                  nowMs, "DHT11 temperature");
  } else {
    sensorFailure(sensors.indoorTemp, "DHT11 temperature");
  }
  if (isfinite(humidity) && humidity >= 10.0f && humidity <= 95.0f) {
    sensorSuccess(sensors.indoorHumidity, humidity, HUMIDITY_EMA_ALPHA,
                  nowMs, "DHT11 humidity");
  } else {
    sensorFailure(sensors.indoorHumidity, "DHT11 humidity");
  }
}

void updateSensors(uint32_t nowMs) {
  if (elapsedMs(nowMs, lastFastSensorMs, FAST_SENSOR_INTERVAL_MS)) {
    lastFastSensorMs = nowMs;
    readFastSensors(nowMs);
  }
  updateDs18b20(nowMs);
  if (elapsedMs(nowMs, lastDhtMs, DHT_INTERVAL_MS)) {
    lastDhtMs = nowMs;
    readDht11(nowMs);
  }
  refreshSensorValidity(nowMs);
}

// =====================================================
// NORMALIZATION AND EEI
// =====================================================

void calculateNormalizedValues() {
  sensors.uv.normalized = normalizeValue(sensors.uv.filteredRaw, UV_MIN, UV_MAX);
  sensors.light.normalized = normalizeValue(sensors.light.filteredRaw,
                                             LIGHT_MIN, LIGHT_MAX);
  sensors.outdoorNorm = sensors.outdoorTemp.health.valid
      ? normalizeValue(sensors.outdoorTemp.value, OUTDOOR_TEMP_MIN, OUTDOOR_TEMP_MAX)
      : 0.0f;
  sensors.indoorNorm = sensors.indoorTemp.health.valid
      ? normalizeValue(sensors.indoorTemp.value, INDOOR_TEMP_MIN, INDOOR_TEMP_MAX)
      : 0.0f;
  sensors.humidityNorm = sensors.indoorHumidity.health.valid
      ? normalizeValue(sensors.indoorHumidity.value, HUMIDITY_MIN, HUMIDITY_MAX)
      : 0.0f;
  sensors.windowAbsoluteNorm = sensors.windowTemp.health.valid
      ? normalizeValue(sensors.windowTemp.value, WINDOW_TEMP_MIN, WINDOW_TEMP_MAX)
      : 0.0f;

  if (sensors.windowTemp.health.valid && sensors.outdoorTemp.health.valid) {
    const float windowDelta = sensors.windowTemp.value - sensors.outdoorTemp.value;
    sensors.windowDeltaNorm = normalizeValue(windowDelta,
                                              WINDOW_DELTA_MIN, WINDOW_DELTA_MAX);
    // Absolute glass temperature is primary; glass-to-shade-air delta is supporting evidence.
    sensors.windowControlNorm = 0.70f * sensors.windowAbsoluteNorm
                              + 0.30f * sensors.windowDeltaNorm;
  } else {
    sensors.windowDeltaNorm = 0.0f;
    sensors.windowControlNorm = sensors.windowAbsoluteNorm;
  }
}

void calculateEEI(uint32_t nowMs) {
  calculateNormalizedValues();

  float weightedSum = 0.0f;
  float validWeight = 0.0f;
  auto addTerm = [&](float value, bool valid, float weight) {
    if (valid && isfinite(value)) {
      weightedSum += clamp01(value) * weight;
      validWeight += weight;
    }
  };

  addTerm(sensors.uv.normalized, sensors.uv.valid, W_UV);
  addTerm(sensors.light.normalized, sensors.light.valid, W_LIGHT);
  addTerm(sensors.outdoorNorm, sensors.outdoorTemp.health.valid, W_OUT_TEMP);
  addTerm(sensors.indoorNorm, sensors.indoorTemp.health.valid, W_IN_TEMP);
  addTerm(sensors.windowControlNorm, sensors.windowTemp.health.valid, W_WINDOW);
  addTerm(sensors.humidityNorm, sensors.indoorHumidity.health.valid, W_HUMIDITY);

  sensors.eeiValid = validWeight >= 0.50f;
  sensors.eei = sensors.eeiValid ? clamp01(weightedSum / validWeight) : 0.0f;
  sensors.lastEeiMs = nowMs;

  const bool nightCandidate = sensors.uv.valid && sensors.light.valid &&
      sensors.uv.normalized <= NIGHT_UV_MAX &&
      sensors.light.normalized <= NIGHT_LIGHT_MAX;
  if (!nightCandidate) {
    nightCandidateActive = false;
    sensors.night = false;
  } else {
    if (!nightCandidateActive) {
      nightCandidateActive = true;
      nightCandidateSinceMs = nowMs;
    }
    if (elapsedMs(nowMs, nightCandidateSinceMs, NIGHT_CONFIRM_TIME_MS)) {
      sensors.night = true;
    }
  }
}

bool automaticSensorsReady() {
  const bool solarInputAvailable = sensors.uv.valid || sensors.light.valid;
  // DHT temperature, window object temperature, and at least one solar channel
  // are critical. Outdoor temperature and humidity are re-weighted if stale.
  return calibrationValid && positionKnown && sensors.eeiValid &&
         solarInputAvailable && sensors.indoorTemp.health.valid &&
         sensors.windowTemp.health.valid;
}

// =====================================================
// PERSISTENT CALIBRATION
// =====================================================

void loadCalibration() {
  preferencesReady = preferences.begin(PREFERENCES_NAMESPACE, false);
  if (!preferencesReady) {
    Serial.println(F("NVS WARNING: Preferences could not be opened; calibration will not persist."));
    calibrationValid = false;
    return;
  }
  const uint32_t version = preferences.getUInt("version", 0UL);
  const long storedSteps = preferences.getLong("deploy", 0L);
  calibrationValid = version == CALIBRATION_STORAGE_VERSION &&
                     storedSteps >= MIN_VALID_CALIBRATION_STEPS &&
                     storedSteps <= CALIBRATION_MEASURE_MAX_STEPS;
  if (calibrationValid) {
    maxTravelSteps = storedSteps;
    Serial.print(F("Saved upper-only deployment limit loaded: DEPLOY_STEPS="));
    Serial.println(maxTravelSteps);
  } else {
    maxTravelSteps = 0;
    Serial.println(F("No valid DEPLOY_STEPS. Run CALIBRATE after homing."));
  }
}

void saveCalibration() {
  if (!preferencesReady || !calibrationValid) return;
  preferences.putUInt("version", CALIBRATION_STORAGE_VERSION);
  preferences.putLong("deploy", maxTravelSteps);
}

void clearCalibration() {
  calibrationValid = false;
  measurementMode = false;
  maxTravelSteps = 0;
  if (preferencesReady) {
    preferences.remove("version");
    preferences.remove("maxsteps");
    preferences.remove("deploy");
  }
  Serial.println(F("Stored DEPLOY_STEPS cleared. Run CALIBRATE before POS or AUTO."));
}

// =====================================================
// MOTOR POSITION AND STATE MACHINE
// =====================================================

long logicalToMotor(long logicalSteps) {
  return MOTOR_DIRECTION_INVERT ? -logicalSteps : logicalSteps;
}

long motorToLogical(long motorSteps) {
  return MOTOR_DIRECTION_INVERT ? -motorSteps : motorSteps;
}

long currentLogicalSteps() {
  return motorToLogical(stepper.currentPosition());
}

float currentPositionPercent() {
  if (!positionKnown || !calibrationValid || maxTravelSteps <= 0) return NAN;
  return constrain(100.0f * static_cast<float>(currentLogicalSteps()) /
                   static_cast<float>(maxTravelSteps), 0.0f, 100.0f);
}

void configureStepper(float maxSpeed, float acceleration) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
  stepper.enableOutputs();
}

void haltMotor(bool releaseOutputs) {
  const long here = stepper.currentPosition();
  stepper.moveTo(here);
  stepper.setCurrentPosition(here);
  if (releaseOutputs) stepper.disableOutputs();
}

void enterError(ErrorCode code, const char* message) {
  haltMotor(true);
  errorCode = code;
  systemState = SystemState::ERROR;
  controlMode = ControlMode::MANUAL;
  positionKnown = false;
  measurementMode = false;
  autoCandidateActive = false;
  Serial.print(F("ERROR: "));
  Serial.print(errorName(code));
  Serial.print(F(" - "));
  Serial.println(message);
  Serial.println(F("Resolve the cause, then issue HOME or RESET. AUTO is disabled."));
}

void emergencyStop(const char* reason) {
  haltMotor(true);
  errorCode = ErrorCode::EMERGENCY_STOP_REQUESTED;
  systemState = SystemState::EMERGENCY_STOP;
  controlMode = ControlMode::MANUAL;
  positionKnown = false;
  measurementMode = false;
  autoCandidateActive = false;
  Serial.print(F("EMERGENCY STOP: "));
  Serial.println(reason);
  Serial.println(F("Position reference is considered lost. Issue HOME before moving again."));
}

void startHoming(HomeReason reason, uint32_t nowMs) {
  errorCode = ErrorCode::NONE;
  homeReason = reason;
  positionKnown = false;
  measurementMode = false;
  systemState = SystemState::HOMING;
  operationStartMs = nowMs;
  operationStartMotorPosition = stepper.currentPosition();
  configureStepper(HOMING_SPEED, HOMING_ACCELERATION);
  // Negative logical motion is always toward the upper endstop.
  stepper.moveTo(stepper.currentPosition() + logicalToMotor(-HOMING_MAX_STEPS));
  Serial.println(F("HOMING: moving slowly toward Upper Endstop."));
}

void completeHoming(uint32_t nowMs) {
  (void)nowMs;
  haltMotor(!HOLD_MOTOR_WHEN_IDLE);
  stepper.setCurrentPosition(logicalToMotor(0L));
  positionKnown = true;
  targetPercent = 0.0f;
  acceptedAutoTargetPercent = 0.0f;
  Serial.println(F("HOMING COMPLETE: Current Steps=0, Position=0%."));

  if (homeReason == HomeReason::CALIBRATION_START) {
    controlMode = ControlMode::MANUAL;
    systemState = SystemState::IDLE;
    measurementMode = true;
    targetPercent = NAN;
    Serial.println(F("MEASUREMENT READY: use JOG 1000/100/50 to move DOWN."));
    Serial.println(F("At the measured physical full-deployment point, issue CAL SET."));
    Serial.println(F("CAL SET will store 92% of the measured steps as DEPLOY_STEPS."));
    return;
  }

  if (controlMode == ControlMode::AUTO && calibrationValid) {
    systemState = SystemState::AUTO_IDLE;
    autoCandidateActive = false;
    autoInvalidTimerActive = false;
  } else {
    controlMode = ControlMode::MANUAL;
    systemState = SystemState::IDLE;
  }

  if (homeReason == HomeReason::BOOT && !AUTO_START_ENABLED) {
    Serial.println(F("READY: MANUAL/IDLE. AUTO will not start until the AUTO command."));
  } else if (homeReason == HomeReason::CALIBRATION_RETURN) {
    Serial.println(F("Calibration return-to-upper complete. MANUAL/IDLE."));
  }
}

void startCalibration(uint32_t nowMs) {
  if (systemState != SystemState::IDLE && systemState != SystemState::AUTO_IDLE) {
    Serial.println(F("CALIBRATE REJECTED: system must be stationary and fault-free."));
    return;
  }
  controlMode = ControlMode::MANUAL;
  movementStartedByAuto = false;
  clearCalibration();
  Serial.println(F("CALIBRATION START: homing to the Upper Endstop first."));
  Serial.println(F("The Lower Endstop is not used in this firmware."));
  startHoming(HomeReason::CALIBRATION_START, nowMs);
}

bool startMeasurementJog(long deltaSteps, uint32_t nowMs) {
  if (!measurementMode || !positionKnown || systemState != SystemState::IDLE) {
    Serial.println(F("JOG REJECTED: run CALIBRATE and wait for MEASUREMENT READY first."));
    return false;
  }
  if (deltaSteps == 0 || labs(deltaSteps) > MAX_MEASUREMENT_JOG_STEPS) {
    Serial.print(F("JOG REJECTED: use a non-zero value from -"));
    Serial.print(MAX_MEASUREMENT_JOG_STEPS);
    Serial.print(F(" to "));
    Serial.println(MAX_MEASUREMENT_JOG_STEPS);
    return false;
  }

  const long current = currentLogicalSteps();
  const long requested = current + deltaSteps;
  if (requested < 0 || requested > CALIBRATION_MEASURE_MAX_STEPS) {
    Serial.println(F("JOG REJECTED: requested position is outside the measurement safety range."));
    return false;
  }

  activeTargetLogicalSteps = requested;
  targetPercent = NAN;
  configureStepper(CALIBRATION_SPEED, CALIBRATION_ACCELERATION);
  stepper.moveTo(logicalToMotor(activeTargetLogicalSteps));
  operationStartMs = nowMs;
  operationStartMotorPosition = stepper.currentPosition();
  systemState = deltaSteps > 0 ? SystemState::MOVING_DOWN : SystemState::MOVING_UP;
  Serial.print(F("MEASUREMENT JOG START: targetSteps="));
  Serial.println(activeTargetLogicalSteps);
  return true;
}

void saveMeasuredDeployment(uint32_t nowMs) {
  if (!measurementMode || !positionKnown || systemState != SystemState::IDLE) {
    Serial.println(F("CAL SET REJECTED: finish a measurement JOG first."));
    return;
  }
  const long measuredSteps = currentLogicalSteps();
  const long safeSteps = lroundf(static_cast<float>(measuredSteps) *
                                 DEPLOY_SAFETY_FACTOR);
  if (measuredSteps < MIN_VALID_CALIBRATION_STEPS ||
      measuredSteps > CALIBRATION_MEASURE_MAX_STEPS ||
      safeSteps < MIN_VALID_CALIBRATION_STEPS) {
    enterError(ErrorCode::CALIBRATION_INVALID,
               "Measured deployment is outside the configured safe range.");
    return;
  }

  maxTravelSteps = safeSteps;
  calibrationValid = true;
  measurementMode = false;
  saveCalibration();
  Serial.println(F("Calibration Complete (Upper Endstop + software lower limit)"));
  Serial.print(F("Measured physical deployment steps = "));
  Serial.println(measuredSteps);
  Serial.print(F("DEPLOY_STEPS saved with 8% safety margin = "));
  Serial.println(maxTravelSteps);
  Serial.println(F("Returning to the Upper Endstop now."));
  controlMode = ControlMode::MANUAL;
  startHoming(HomeReason::CALIBRATION_RETURN, nowMs);
}

bool moveBlindToPercent(float percent, bool fromAuto, uint32_t nowMs) {
  if (!calibrationValid) {
    Serial.println(F("MOVE REJECTED: No calibration. Run CALIBRATE first."));
    return false;
  }
  if (!positionKnown) {
    Serial.println(F("MOVE REJECTED: Position unknown. Run HOME first."));
    return false;
  }
  if (systemState == SystemState::ERROR ||
      systemState == SystemState::EMERGENCY_STOP) {
    Serial.println(F("MOVE REJECTED: Clear the fault with HOME first."));
    return false;
  }

  percent = constrain(percent, 0.0f, 100.0f);
  movementStartedByAuto = fromAuto;
  targetPercent = percent;

  if (percent <= 0.01f) {
    startHoming(fromAuto ? HomeReason::AUTO_POSITION_ZERO : HomeReason::COMMAND,
                nowMs);
    return true;
  }

  const long currentLogical = currentLogicalSteps();
  activeTargetLogicalSteps = lroundf(static_cast<float>(maxTravelSteps) *
                                     percent / 100.0f);
  if (activeTargetLogicalSteps == currentLogical) {
    Serial.println(F("MOVE: already at the requested step position."));
    return true;
  }

  configureStepper(NORMAL_MOVE_SPEED, NORMAL_MOVE_ACCELERATION);
  stepper.moveTo(logicalToMotor(activeTargetLogicalSteps));
  operationStartMs = nowMs;
  operationStartMotorPosition = stepper.currentPosition();
  systemState = activeTargetLogicalSteps > currentLogical
      ? SystemState::MOVING_DOWN : SystemState::MOVING_UP;
  Serial.print(F("MOVE START: target="));
  Serial.print(percent, 1);
  Serial.print(F("%, targetSteps="));
  Serial.println(activeTargetLogicalSteps);
  return true;
}

void startManualDown(uint32_t nowMs) {
  if (!positionKnown) {
    Serial.println(F("DOWN REJECTED: Run HOME first."));
    return;
  }
  controlMode = ControlMode::MANUAL;
  movementStartedByAuto = false;
  if (calibrationValid) {
    moveBlindToPercent(100.0f, false, nowMs);
    return;
  }
  Serial.println(F("DOWN REJECTED: no DEPLOY_STEPS. Run CALIBRATE, JOG, and CAL SET first."));
}

void finishNormalMove(uint32_t nowMs) {
  (void)nowMs;
  haltMotor(!HOLD_MOTOR_WHEN_IDLE);
  if (measurementMode) {
    systemState = SystemState::IDLE;
    Serial.print(F("MEASUREMENT JOG COMPLETE: measuredSteps="));
    Serial.println(currentLogicalSteps());
    Serial.println(F("Continue with JOG, or issue CAL SET at the physical deployment point."));
    return;
  }
  const float actualPercent = currentPositionPercent();
  if (isfinite(actualPercent)) targetPercent = actualPercent;
  systemState = controlMode == ControlMode::AUTO
      ? SystemState::AUTO_IDLE : SystemState::IDLE;
  Serial.print(F("MOVE COMPLETE: steps="));
  Serial.print(currentLogicalSteps());
  Serial.print(F(", position="));
  printFloatOrNA(actualPercent, isfinite(actualPercent), 1);
  Serial.println(F("%."));
}

void updateMotionState(uint32_t nowMs) {
  switch (systemState) {
    case SystemState::HOMING: {
      if (upperEndstop.stablePressed) {
        completeHoming(nowMs);
        return;
      }
      const long moved = labs(stepper.currentPosition() - operationStartMotorPosition);
      if (elapsedMs(nowMs, operationStartMs, HOMING_TIMEOUT_MS) ||
          moved >= HOMING_MAX_STEPS || stepper.distanceToGo() == 0) {
        enterError(ErrorCode::HOMING_TIMEOUT,
                   "Upper Endstop was not reached within the homing limit.");
        return;
      }
      stepper.run();
      break;
    }

    case SystemState::MOVING_UP:
    case SystemState::MOVING_DOWN: {
      const bool movingUp = systemState == SystemState::MOVING_UP;
      if (movingUp && upperEndstop.stablePressed) {
        haltMotor(!HOLD_MOTOR_WHEN_IDLE);
        stepper.setCurrentPosition(logicalToMotor(0L));
        positionKnown = true;
        if (measurementMode) {
          systemState = SystemState::IDLE;
          Serial.println(F("MEASUREMENT JOG: Upper Endstop reached; measuredSteps=0."));
          return;
        }
        const bool expectedUpper = !isfinite(targetPercent) || targetPercent <= 0.5f;
        if (!expectedUpper) {
          enterError(ErrorCode::POSITION_INVALID,
                     "Upper Endstop was reached before the commanded intermediate position.");
        } else {
          finishNormalMove(nowMs);
        }
        return;
      }

      if (elapsedMs(nowMs, operationStartMs, MOVE_TIMEOUT_MS)) {
        enterError(ErrorCode::MOVE_TIMEOUT,
                   "The normal movement exceeded MOVE_TIMEOUT_MS.");
        return;
      }

      if (calibrationValid && !measurementMode) {
        const long logical = currentLogicalSteps();
        if (logical < -POSITION_ERROR_TOLERANCE_STEPS ||
            logical > maxTravelSteps + POSITION_ERROR_TOLERANCE_STEPS) {
          enterError(ErrorCode::POSITION_INVALID,
                     "Calculated position is outside the calibrated travel range.");
          return;
        }
      } else if (measurementMode) {
        const long logical = currentLogicalSteps();
        if (logical < 0 || logical > CALIBRATION_MEASURE_MAX_STEPS) {
          enterError(ErrorCode::POSITION_INVALID,
                     "Measurement position exceeded its software safety range.");
          return;
        }
      }

      stepper.run();
      if (stepper.distanceToGo() == 0) {
        finishNormalMove(nowMs);
      }
      break;
    }

    default:
      break;
  }
}

// =====================================================
// AUTOMATIC CONTROL
// =====================================================

float calculateTargetPosition(float eei, float referencePercent) {
  if (sensors.night) return 0.0f;

  if (EEI_CONTINUOUS_POSITION) {
    const float continuous = constrain(eei * 100.0f, 0.0f, 100.0f);
    return fabsf(continuous - referencePercent) < EEI_HYSTERESIS * 100.0f
        ? referencePercent : continuous;
  }

  const float thresholds[4] = {
      EEI_LEVEL_1, EEI_LEVEL_2, EEI_LEVEL_3, EEI_LEVEL_4};
  int level = constrain(static_cast<int>(lroundf(referencePercent / 25.0f)), 0, 4);

  // A level rises only after its upper boundary plus hysteresis is crossed,
  // and falls only below its lower boundary minus hysteresis.
  while (level < 4 && eei >= thresholds[level] + EEI_HYSTERESIS) ++level;
  while (level > 0 && eei < thresholds[level - 1] - EEI_HYSTERESIS) --level;
  return static_cast<float>(level * 25);
}

void updateAutomaticControl(uint32_t nowMs) {
  if (controlMode != ControlMode::AUTO ||
      systemState != SystemState::AUTO_IDLE) return;
  if (!elapsedMs(nowMs, lastAutoControlMs, AUTO_CONTROL_INTERVAL_MS)) return;
  lastAutoControlMs = nowMs;

  if (!automaticSensorsReady()) {
    autoCandidateActive = false;
    if (!autoInvalidTimerActive) {
      autoInvalidTimerActive = true;
      autoInvalidSinceMs = nowMs;
      Serial.println(F("AUTO WARNING: waiting for fresh critical sensor data."));
    } else if (elapsedMs(nowMs, autoInvalidSinceMs, AUTO_SENSOR_GRACE_MS)) {
      controlMode = ControlMode::MANUAL;
      systemState = SystemState::IDLE;
      autoInvalidTimerActive = false;
      Serial.println(F("AUTO DISABLED: critical sensor data remained unavailable."));
    }
    return;
  }
  autoInvalidTimerActive = false;

  const float candidate = calculateTargetPosition(sensors.eei,
                                                   acceptedAutoTargetPercent);
  const float current = currentPositionPercent();
  if (!isfinite(current)) {
    enterError(ErrorCode::POSITION_INVALID,
               "AUTO cannot calculate the current position percentage.");
    return;
  }

  if (fabsf(candidate - current) < MIN_POSITION_CHANGE_PERCENT) {
    autoCandidateActive = false;
    return;
  }

  if (!autoCandidateActive || fabsf(candidate - pendingAutoTargetPercent) > 0.5f) {
    pendingAutoTargetPercent = candidate;
    autoCandidateSinceMs = nowMs;
    autoCandidateActive = true;
    Serial.print(F("AUTO CANDIDATE: "));
    Serial.print(candidate, 1);
    Serial.println(F("%; waiting for CONTROL_STABLE_TIME_MS."));
    return;
  }

  if (elapsedMs(nowMs, autoCandidateSinceMs, CONTROL_STABLE_TIME_MS)) {
    acceptedAutoTargetPercent = pendingAutoTargetPercent;
    autoCandidateActive = false;
    moveBlindToPercent(acceptedAutoTargetPercent, true, nowMs);
  }
}

// =====================================================
// SERIAL OUTPUT
// =====================================================

const char* stateName(SystemState state) {
  switch (state) {
    case SystemState::STARTUP: return "STARTUP";
    case SystemState::HOMING: return "HOMING";
    case SystemState::IDLE: return "IDLE";
    case SystemState::MOVING_UP: return "MOVING_UP";
    case SystemState::MOVING_DOWN: return "MOVING_DOWN";
    case SystemState::AUTO_IDLE: return "AUTO_IDLE";
    case SystemState::ERROR: return "ERROR";
    case SystemState::EMERGENCY_STOP: return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}

const char* modeName(ControlMode mode) {
  return mode == ControlMode::AUTO ? "AUTO" : "MANUAL";
}

const char* errorName(ErrorCode code) {
  switch (code) {
    case ErrorCode::NONE: return "NONE";
    case ErrorCode::HOMING_TIMEOUT: return "HOMING_TIMEOUT";
    case ErrorCode::MOVE_TIMEOUT: return "MOVE_TIMEOUT";
    case ErrorCode::POSITION_INVALID: return "POSITION_INVALID";
    case ErrorCode::CALIBRATION_INVALID: return "CALIBRATION_INVALID";
    case ErrorCode::EMERGENCY_STOP_REQUESTED: return "EMERGENCY_STOP_REQUESTED";
  }
  return "UNKNOWN";
}

void printFloatOrNA(float value, bool valid, uint8_t decimals) {
  if (valid && isfinite(value)) Serial.print(value, decimals);
  else Serial.print(F("NA"));
}

void printSensors() {
  Serial.println(F("=== SENSORS ==="));
  Serial.print(F("GUVA: raw=")); Serial.print(sensors.uv.raw);
  Serial.print(F(", filtered=")); Serial.print(sensors.uv.filteredRaw, 1);
  Serial.print(F(", voltage~=")); Serial.print(sensors.uv.voltage, 3);
  Serial.print(F("V, norm=")); Serial.println(sensors.uv.normalized, 3);

  Serial.print(F("LIGHT: raw=")); Serial.print(sensors.light.raw);
  Serial.print(F(", filtered=")); Serial.print(sensors.light.filteredRaw, 1);
  Serial.print(F(", voltage~=")); Serial.print(sensors.light.voltage, 3);
  Serial.print(F("V, norm=")); Serial.println(sensors.light.normalized, 3);

  Serial.print(F("DS18B20 outdoor="));
  printFloatOrNA(sensors.outdoorTemp.value, sensors.outdoorTemp.health.valid, 2);
  Serial.print(F("C, norm="));
  printFloatOrNA(sensors.outdoorNorm, sensors.outdoorTemp.health.valid, 3);
  Serial.print(F(", failures="));
  Serial.println(sensors.outdoorTemp.health.totalFailures);

  Serial.print(F("DHT11 indoor="));
  printFloatOrNA(sensors.indoorTemp.value, sensors.indoorTemp.health.valid, 2);
  Serial.print(F("C, norm="));
  printFloatOrNA(sensors.indoorNorm, sensors.indoorTemp.health.valid, 3);
  Serial.print(F(", failures="));
  Serial.println(sensors.indoorTemp.health.totalFailures);

  Serial.print(F("DHT11 humidity="));
  printFloatOrNA(sensors.indoorHumidity.value, sensors.indoorHumidity.health.valid, 1);
  Serial.print(F("%, norm="));
  printFloatOrNA(sensors.humidityNorm, sensors.indoorHumidity.health.valid, 3);
  Serial.print(F(", failures="));
  Serial.println(sensors.indoorHumidity.health.totalFailures);

  Serial.print(F("MLX90614 object/window="));
  printFloatOrNA(sensors.windowTemp.value, sensors.windowTemp.health.valid, 2);
  Serial.print(F("C, absNorm="));
  printFloatOrNA(sensors.windowAbsoluteNorm, sensors.windowTemp.health.valid, 3);
  Serial.print(F(", deltaNorm="));
  printFloatOrNA(sensors.windowDeltaNorm,
                 sensors.windowTemp.health.valid && sensors.outdoorTemp.health.valid, 3);
  Serial.print(F(", failures="));
  Serial.println(sensors.windowTemp.health.totalFailures);

  Serial.print(F("MLX90614 ambient="));
  printFloatOrNA(sensors.mlxAmbientTemp.value,
                 sensors.mlxAmbientTemp.health.valid, 2);
  Serial.print(F("C, failures="));
  Serial.println(sensors.mlxAmbientTemp.health.totalFailures);

  Serial.print(F("EEI="));
  printFloatOrNA(sensors.eei, sensors.eeiValid, 3);
  Serial.print(F(", night="));
  Serial.println(sensors.night ? F("YES") : F("NO"));
}

void printStatus() {
  printSensors();
  Serial.println(F("=== POSITION / SYSTEM ==="));
  Serial.print(F("Current Steps="));
  if (positionKnown) Serial.println(currentLogicalSteps());
  else Serial.println(F("UNKNOWN"));
  Serial.print(F("DEPLOY_STEPS="));
  if (calibrationValid) Serial.println(maxTravelSteps);
  else Serial.println(F("UNCALIBRATED"));
  Serial.print(F("Current Position="));
  const float current = currentPositionPercent();
  printFloatOrNA(current, isfinite(current), 1);
  Serial.println('%');
  Serial.print(F("Target Position="));
  printFloatOrNA(targetPercent, isfinite(targetPercent), 1);
  Serial.println('%');
  Serial.print(F("Upper Endstop=")); Serial.println(upperEndstop.stablePressed ? 1 : 0);
  Serial.print(F("Measurement Mode=")); Serial.println(measurementMode ? F("YES") : F("NO"));
  Serial.print(F("Mode=")); Serial.println(modeName(controlMode));
  Serial.print(F("State=")); Serial.println(stateName(systemState));
  Serial.print(F("Error=")); Serial.println(errorName(errorCode));
  Serial.print(F("AUTO data ready=")); Serial.println(automaticSensorsReady() ? 1 : 0);
  Serial.print(F("CSV logging=")); Serial.println(csvLoggingEnabled ? F("ON") : F("OFF"));
  Serial.println();
}

void printCsvHeader() {
  Serial.println(F("timestamp_ms,uv_raw,uv_voltage,uv_norm,light_raw,light_voltage,light_norm,outdoor_temp,indoor_temp,humidity,window_temp,mlx_ambient,window_norm,eei,current_steps,deploy_steps,current_percent,target_percent,upper,mode,state,error"));
}

void printCsvValue(float value, bool valid, uint8_t decimals) {
  if (valid && isfinite(value)) Serial.print(value, decimals);
}

void printCsvRow(uint32_t nowMs) {
  Serial.print(nowMs); Serial.print(',');
  Serial.print(sensors.uv.raw); Serial.print(',');
  Serial.print(sensors.uv.voltage, 3); Serial.print(',');
  Serial.print(sensors.uv.normalized, 3); Serial.print(',');
  Serial.print(sensors.light.raw); Serial.print(',');
  Serial.print(sensors.light.voltage, 3); Serial.print(',');
  Serial.print(sensors.light.normalized, 3); Serial.print(',');
  printCsvValue(sensors.outdoorTemp.value, sensors.outdoorTemp.health.valid, 2); Serial.print(',');
  printCsvValue(sensors.indoorTemp.value, sensors.indoorTemp.health.valid, 2); Serial.print(',');
  printCsvValue(sensors.indoorHumidity.value, sensors.indoorHumidity.health.valid, 1); Serial.print(',');
  printCsvValue(sensors.windowTemp.value, sensors.windowTemp.health.valid, 2); Serial.print(',');
  printCsvValue(sensors.mlxAmbientTemp.value, sensors.mlxAmbientTemp.health.valid, 2); Serial.print(',');
  printCsvValue(sensors.windowControlNorm, sensors.windowTemp.health.valid, 3); Serial.print(',');
  printCsvValue(sensors.eei, sensors.eeiValid, 3); Serial.print(',');
  if (positionKnown) Serial.print(currentLogicalSteps()); Serial.print(',');
  if (calibrationValid) Serial.print(maxTravelSteps); Serial.print(',');
  const float current = currentPositionPercent();
  printCsvValue(current, isfinite(current), 1); Serial.print(',');
  printCsvValue(targetPercent, isfinite(targetPercent), 1); Serial.print(',');
  Serial.print(upperEndstop.stablePressed ? 1 : 0); Serial.print(',');
  Serial.print(modeName(controlMode)); Serial.print(',');
  Serial.print(stateName(systemState)); Serial.print(',');
  Serial.println(errorName(errorCode));
}

void printHelp() {
  Serial.println(F("Commands (case-insensitive):"));
  Serial.println(F("  STATUS | SENSORS | ENDSTOPS | HELP"));
  Serial.println(F("  UP | DOWN | STOP | HOME | RESET | CALIBRATE"));
  Serial.println(F("  JOG -2000..2000   (measurement mode only; + is DOWN)"));
  Serial.println(F("  CAL SET      (save 92% of measured steps as DEPLOY_STEPS)"));
  Serial.println(F("  POS 0..100   (example: POS 37)"));
  Serial.println(F("  AUTO | MANUAL"));
  Serial.println(F("  LOG ON | LOG OFF"));
  Serial.println(F("  REPORT ON | REPORT OFF"));
  Serial.println(F("  CAL CLEAR    (erase saved DEPLOY_STEPS)"));
}

// =====================================================
// SERIAL COMMANDS
// =====================================================

void processCommand(char* command, uint32_t nowMs) {
  while (*command == ' ' || *command == '\t') ++command;
  char* end = command + strlen(command);
  while (end > command && (end[-1] == ' ' || end[-1] == '\t')) --end;
  *end = '\0';
  for (char* p = command; *p; ++p) {
    *p = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
  }
  if (*command == '\0') return;

  if (strcmp(command, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(command, "SENSORS") == 0) {
    printSensors();
  } else if (strcmp(command, "ENDSTOPS") == 0) {
    Serial.print(F("Upper=")); Serial.println(upperEndstop.stablePressed ? 1 : 0);
  } else if (strcmp(command, "HELP") == 0) {
    printHelp();
  } else if (strcmp(command, "STOP") == 0) {
    emergencyStop("Serial STOP command");
  } else if (strcmp(command, "HOME") == 0 || strcmp(command, "RESET") == 0) {
    controlMode = ControlMode::MANUAL;
    errorCode = ErrorCode::NONE;
    startHoming(HomeReason::COMMAND, nowMs);
  } else if (strcmp(command, "CALIBRATE") == 0) {
    startCalibration(nowMs);
  } else if (strncmp(command, "JOG ", 4) == 0) {
    char* parseEnd = nullptr;
    const long requested = strtol(command + 4, &parseEnd, 10);
    while (parseEnd && (*parseEnd == ' ' || *parseEnd == '\t')) ++parseEnd;
    if (parseEnd == command + 4 || (parseEnd && *parseEnd != '\0')) {
      Serial.println(F("JOG REJECTED: example commands are JOG 1000 and JOG -100."));
    } else {
      startMeasurementJog(requested, nowMs);
    }
  } else if (strcmp(command, "CAL SET") == 0) {
    saveMeasuredDeployment(nowMs);
  } else if (strcmp(command, "UP") == 0) {
    controlMode = ControlMode::MANUAL;
    errorCode = ErrorCode::NONE;
    startHoming(HomeReason::COMMAND, nowMs);
  } else if (strcmp(command, "DOWN") == 0) {
    if (systemState == SystemState::ERROR ||
        systemState == SystemState::EMERGENCY_STOP) {
      Serial.println(F("DOWN REJECTED: run HOME first."));
    } else {
      startManualDown(nowMs);
    }
  } else if (strncmp(command, "POS ", 4) == 0) {
    char* parseEnd = nullptr;
    const float requested = strtof(command + 4, &parseEnd);
    while (parseEnd && (*parseEnd == ' ' || *parseEnd == '\t')) ++parseEnd;
    if (parseEnd == command + 4 || (parseEnd && *parseEnd != '\0') ||
        !isfinite(requested) || requested < 0.0f || requested > 100.0f) {
      Serial.println(F("POS REJECTED: use POS followed by a value from 0 to 100."));
    } else {
      controlMode = ControlMode::MANUAL;
      moveBlindToPercent(requested, false, nowMs);
    }
  } else if (strcmp(command, "AUTO") == 0) {
    if (systemState != SystemState::IDLE && systemState != SystemState::AUTO_IDLE) {
      Serial.println(F("AUTO REJECTED: system must be stationary and fault-free."));
    } else if (!calibrationValid) {
      Serial.println(F("AUTO REJECTED: run CALIBRATE first."));
    } else if (!automaticSensorsReady()) {
      Serial.println(F("AUTO REJECTED: critical sensor data is not ready. Check STATUS."));
    } else {
      controlMode = ControlMode::AUTO;
      systemState = SystemState::AUTO_IDLE;
      acceptedAutoTargetPercent = currentPositionPercent();
      targetPercent = acceptedAutoTargetPercent;
      autoCandidateActive = false;
      autoInvalidTimerActive = false;
      lastAutoControlMs = nowMs - AUTO_CONTROL_INTERVAL_MS;
      Serial.println(F("AUTO ENABLED."));
    }
  } else if (strcmp(command, "MANUAL") == 0) {
    if (systemState == SystemState::MOVING_UP ||
        systemState == SystemState::MOVING_DOWN) {
      haltMotor(!HOLD_MOTOR_WHEN_IDLE);
    }
    controlMode = ControlMode::MANUAL;
    if (systemState == SystemState::AUTO_IDLE ||
        systemState == SystemState::MOVING_UP ||
        systemState == SystemState::MOVING_DOWN) {
      systemState = SystemState::IDLE;
    }
    autoCandidateActive = false;
    Serial.println(F("MANUAL ENABLED."));
  } else if (strcmp(command, "LOG ON") == 0) {
    csvLoggingEnabled = true;
    lastLogMs = nowMs - LOG_INTERVAL_MS;
    printCsvHeader();
  } else if (strcmp(command, "LOG OFF") == 0) {
    csvLoggingEnabled = false;
    Serial.println(F("CSV logging OFF."));
  } else if (strcmp(command, "REPORT ON") == 0) {
    periodicStatusEnabled = true;
    lastStatusMs = nowMs;
    Serial.println(F("Periodic STATUS reports ON."));
  } else if (strcmp(command, "REPORT OFF") == 0) {
    periodicStatusEnabled = false;
    Serial.println(F("Periodic STATUS reports OFF."));
  } else if (strcmp(command, "CAL CLEAR") == 0) {
    if (systemState == SystemState::IDLE || systemState == SystemState::AUTO_IDLE) {
      controlMode = ControlMode::MANUAL;
      systemState = SystemState::IDLE;
      clearCalibration();
    } else {
      Serial.println(F("CAL CLEAR REJECTED: stop movement first."));
    }
  } else {
    Serial.print(F("UNKNOWN COMMAND: "));
    Serial.println(command);
    printHelp();
  }
}

void processSerialInput(uint32_t nowMs) {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (serialLength > 0) {
        serialBuffer[serialLength] = '\0';
        processCommand(serialBuffer, nowMs);
        serialLength = 0;
      }
    } else if (serialLength < sizeof(serialBuffer) - 1U) {
      serialBuffer[serialLength++] = c;
    } else {
      serialLength = 0;
      Serial.println(F("COMMAND ERROR: input line too long; buffer cleared."));
    }
  }
}

// =====================================================
// ARDUINO ENTRY POINTS
// =====================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t nowMs = millis();
  Serial.println();
  Serial.println(F("Smart Variable Reflective Shade V3 UPPER-ONLY (DS18B20=D2)"));
  Serial.println(F("[1] Serial started at 115200 baud."));

  Wire.begin();
  Serial.println(F("[2] I2C initialized on A4/SDA and A5/SCL."));

  initializeSensors(nowMs);

  beginEndstop(upperEndstop, PIN_UPPER_ENDSTOP, nowMs);
  Serial.print(F("[8] Upper Endstop initialized. Upper="));
  Serial.println(upperEndstop.stablePressed ? 1 : 0);
  Serial.println(F("[8] D5/Lower Endstop is unused; lower travel uses DEPLOY_STEPS."));

  configureStepper(NORMAL_MOVE_SPEED, NORMAL_MOVE_ACCELERATION);
  stepper.disableOutputs();
  Serial.println(F("[9] 28BYJ-48/ULN2003 stepper initialized."));

  loadCalibration();
  Serial.println(F("[10] Initial safety check."));
  controlMode = (AUTO_START_ENABLED && calibrationValid)
      ? ControlMode::AUTO : ControlMode::MANUAL;
  movementStartedByAuto = controlMode == ControlMode::AUTO;
  Serial.println(F("[11] Boot homing will start now."));
  startHoming(HomeReason::BOOT, nowMs);

  lastStatusMs = nowMs;
  lastLogMs = nowMs;
  sensors.lastEeiMs = nowMs - EEI_INTERVAL_MS;
  printHelp();
}

void loop() {
  const uint32_t nowMs = millis();

  // STOP and other commands are serviced before another motor step.
  processSerialInput(nowMs);
  updateEndstops(nowMs);
  updateSensors(nowMs);

  updateMotionState(nowMs);

  if (elapsedMs(nowMs, sensors.lastEeiMs, EEI_INTERVAL_MS)) {
    calculateEEI(nowMs);
  }
  updateAutomaticControl(nowMs);

  if (periodicStatusEnabled && elapsedMs(nowMs, lastStatusMs, STATUS_INTERVAL_MS)) {
    lastStatusMs = nowMs;
    printStatus();
  }
  if (csvLoggingEnabled && elapsedMs(nowMs, lastLogMs, LOG_INTERVAL_MS)) {
    lastLogMs = nowMs;
    printCsvRow(nowMs);
  }
}
