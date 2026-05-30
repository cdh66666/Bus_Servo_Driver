#ifndef BOARD_SERVO_HARDWARE_H
#define BOARD_SERVO_HARDWARE_H

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include "HLS3606Emu.h"

static constexpr int PIN_MOTOR_EN = 1;
static constexpr int PIN_MOTOR_PH = 2;
static constexpr int PIN_MOTOR_NFAULT = 3;
static constexpr int PIN_MOTOR_CURRENT = 4;
static constexpr int PIN_I2C_SDA = 8;
static constexpr int PIN_I2C_SCL = 9;

static constexpr uint8_t MT6701_I2C_ADDR = 0x06;
static constexpr uint8_t AS5600_I2C_ADDR = 0x36;
static constexpr uint32_t MT6701_I2C_CLOCK = 100000;
static constexpr uint32_t MOTOR_PWM_FREQ = 20000;
static constexpr uint8_t MOTOR_PWM_RES_BITS = 10;
static constexpr uint16_t MOTOR_PWM_MAX = (1U << MOTOR_PWM_RES_BITS) - 1U;

static constexpr float IPROPI_RESISTOR_OHMS = 2500.0f;
static constexpr float IPROPI_UA_PER_A = 1000.0f;
static constexpr float CURRENT_MA_PER_MV =
    1000.0f / (IPROPI_RESISTOR_OHMS * (IPROPI_UA_PER_A / 1000000.0f) * 1000.0f);
static constexpr int16_t HARDWARE_CURRENT_LIMIT_MA = 1320;
static constexpr float FEETECH_SPEED_COUNTS_PER_UNIT = 56.0f;
static constexpr int32_t HLS_MIN_POSITION_COMMAND = -30000;
static constexpr int32_t HLS_MAX_POSITION_COMMAND = 30000;

static constexpr uint8_t HLS_ERROR_FAULT = 0x01;
static constexpr uint8_t HLS_ERROR_ENCODER = 0x02;
static constexpr uint8_t HLS_ERROR_OVERCURRENT = 0x04;

static constexpr uint8_t HLS_DIAG_RAW14_L = 0x57;
static constexpr uint8_t HLS_DIAG_MULTI_POS_0 = 0x59;
static constexpr uint8_t HLS_DIAG_FIRST_I2C = 0x5D;
static constexpr uint8_t HLS_DIAG_ACTIVE_I2C = 0x5E;
static constexpr uint8_t HLS_DIAG_ENCODER_STATUS = 0x5F;

template <typename T>
static T clampValue(T value, T low, T high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

class DRV8876Motor {
public:
  explicit DRV8876Motor(uint8_t pwmChannel = 0) : _pwmChannel(pwmChannel) {}

  void begin() {
    pinMode(PIN_MOTOR_PH, OUTPUT);
    pinMode(PIN_MOTOR_EN, OUTPUT);
    pinMode(PIN_MOTOR_NFAULT, INPUT_PULLUP);
    ledcSetup(_pwmChannel, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_EN, _pwmChannel);
    coast();
  }

  void drive(float effort) {
    effort = constrain(effort, -1.0f, 1.0f);
    if (fabsf(effort) < 0.01f) {
      coast();
      return;
    }

    digitalWrite(PIN_MOTOR_PH, effort >= 0.0f ? HIGH : LOW);
    const uint16_t duty = static_cast<uint16_t>(fabsf(effort) * MOTOR_PWM_MAX);
    ledcWrite(_pwmChannel, clampValue<uint16_t>(duty, 0, MOTOR_PWM_MAX));
    _lastEffort = effort;
  }

  void coast() {
    ledcWrite(_pwmChannel, 0);
    _lastEffort = 0.0f;
  }

  bool faulted() const {
    return digitalRead(PIN_MOTOR_NFAULT) == LOW;
  }

  float lastEffort() const { return _lastEffort; }

private:
  uint8_t _pwmChannel;
  float _lastEffort = 0.0f;
};

class MT6701Encoder {
public:
  void begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(MT6701_I2C_CLOCK);
    scanBus();
    _online = readRaw(_lastRaw);
    _extended = _lastRaw;
    _lastSampleUs = micros();
  }

  bool update() {
    uint16_t raw = 0;
    if (!readRaw(raw)) {
      _online = false;
      return false;
    }

    const int16_t diff = unwrapDiff(raw, _lastRaw);
    const uint32_t now = micros();
    const uint32_t dt = now - _lastSampleUs;
    if (dt > 0) {
      _speedAccumRaw += diff;
      _speedAccumUs += dt;
      if (_speedAccumUs >= 8000) {
        const float windowSpeed =
            ((static_cast<float>(_speedAccumRaw) / 4.0f) * 1000000.0f) /
            static_cast<float>(_speedAccumUs);
        const float blend = fabsf(windowSpeed) > 5.0f ? 0.35f : 0.12f;
        _speed4096PerSec = (_speed4096PerSec * (1.0f - blend)) + (windowSpeed * blend);
        _speedAccumRaw = 0;
        _speedAccumUs = 0;
      }
      if (diff == 0 && now - _lastMotionRawUs > 20000) _speed4096PerSec *= 0.94f;
      if (diff != 0) _lastMotionRawUs = now;
    }

    _extended += diff;
    _lastRaw = raw;
    _lastSampleUs = now;
    _online = true;
    return true;
  }

  bool online() const { return _online; }

  int32_t position4096() const {
    return _extended >> 2;
  }

  void alignToSavedPosition4096(int32_t savedPosition) {
    if (!_online) return;
    const int32_t single = singleTurn4096();
    const int32_t savedSingle = positiveModulo(savedPosition, 4096);
    int32_t aligned = savedPosition + (single - savedSingle);
    if (aligned - savedPosition > 2048) aligned -= 4096;
    if (aligned - savedPosition < -2048) aligned += 4096;
    _extended = aligned << 2;
  }

  uint16_t singleTurn4096() const {
    return static_cast<uint16_t>((_lastRaw >> 2) & 0x0FFF);
  }

  int16_t speed4096PerSec() const {
    return static_cast<int16_t>(constrain(_speed4096PerSec, -32767.0f, 32767.0f));
  }

  uint8_t firstAddress() const { return _firstAddress; }
  uint8_t activeAddress() const { return _activeAddress; }
  uint8_t lastStatus() const { return _lastStatus; }
  uint8_t lastBytes() const { return _lastBytes; }
  uint16_t lastRaw14() const { return _lastRaw; }

private:
  bool _online = false;
  uint16_t _lastRaw = 0;
  int32_t _extended = 0;
  uint32_t _lastSampleUs = 0;
  uint32_t _lastMotionRawUs = 0;
  uint32_t _speedAccumUs = 0;
  int32_t _speedAccumRaw = 0;
  float _speed4096PerSec = 0.0f;
  uint8_t _firstAddress = 0;
  uint8_t _activeAddress = MT6701_I2C_ADDR;
  uint8_t _lastStatus = 0xFF;
  uint8_t _lastBytes = 0;

  static int32_t positiveModulo(int32_t value, int32_t modulo) {
    int32_t result = value % modulo;
    if (result < 0) result += modulo;
    return result;
  }

  static int16_t unwrapDiff(uint16_t current, uint16_t previous) {
    int16_t diff = static_cast<int16_t>(current) - static_cast<int16_t>(previous);
    if (diff > 8192) diff -= 16384;
    if (diff < -8192) diff += 16384;
    return diff;
  }

  void scanBus() {
    _firstAddress = 0;
    for (uint8_t addr = 1; addr < 0x7F; ++addr) {
      if (probeAddress(addr)) {
        _firstAddress = addr;
        break;
      }
      delayMicroseconds(50);
    }

    if (probeAddress(MT6701_I2C_ADDR)) {
      _activeAddress = MT6701_I2C_ADDR;
    } else if (probeAddress(AS5600_I2C_ADDR)) {
      _activeAddress = AS5600_I2C_ADDR;
    } else {
      _activeAddress = _firstAddress;
    }
  }

  bool probeAddress(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission(true) == 0;
  }

  bool readRegister(uint8_t addr, uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    const uint8_t txErr = Wire.endTransmission(true);
    if (txErr != 0) {
      _lastStatus = 0x40 | txErr;
      _lastBytes = 0;
      return false;
    }
    _lastBytes = Wire.requestFrom(addr, static_cast<uint8_t>(1));
    if (_lastBytes != 1) {
      _lastStatus = 0x50 | (_lastBytes & 0x0F);
      return false;
    }
    value = Wire.read();
    return true;
  }

  bool readBurst(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    const uint8_t txErr = Wire.endTransmission(false);
    if (txErr != 0) {
      _lastStatus = 0x60 | txErr;
      _lastBytes = 0;
      return false;
    }
    _lastBytes = Wire.requestFrom(addr, len, static_cast<uint8_t>(true));
    if (_lastBytes != len) {
      _lastStatus = 0x70 | (_lastBytes & 0x0F);
      return false;
    }
    for (uint8_t i = 0; i < len; ++i) {
      buffer[i] = Wire.read();
    }
    return true;
  }

  bool readMt6701Raw(uint16_t &raw) {
    uint8_t data[2] = {};
    if (readBurst(_activeAddress, 0x03, data, sizeof(data))) {
      raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
      raw >>= 2;
      raw &= 0x3FFF;
      _lastStatus = 0;
      return true;
    }

    uint8_t high = 0;
    uint8_t low = 0;
    if (!readRegister(_activeAddress, 0x03, high)) return false;
    if (!readRegister(_activeAddress, 0x04, low)) return false;
    raw = (static_cast<uint16_t>(high) << 8) | low;
    raw >>= 2;
    raw &= 0x3FFF;
    _lastStatus = 1;
    _lastBytes = 2;
    return true;
  }

  bool readAs5600Raw(uint16_t &raw) {
    uint8_t data[2] = {};
    if (!readBurst(AS5600_I2C_ADDR, 0x0C, data, sizeof(data))) return false;
    const uint16_t raw12 = ((static_cast<uint16_t>(data[0]) & 0x0F) << 8) | data[1];
    raw = (raw12 & 0x0FFF) << 2;
    _lastStatus = 2;
    _lastBytes = 2;
    return true;
  }

  bool readRaw(uint16_t &raw) {
    if (_activeAddress == AS5600_I2C_ADDR) {
      if (readAs5600Raw(raw)) return true;
      return readMt6701Raw(raw);
    }
    if (readMt6701Raw(raw)) return true;
    if (_firstAddress == AS5600_I2C_ADDR || probeAddress(AS5600_I2C_ADDR)) {
      _activeAddress = AS5600_I2C_ADDR;
      return readAs5600Raw(raw);
    }
    return false;
  }
};

class BoardServoHardware {
public:
  explicit BoardServoHardware(HLS3606Emu &servo, uint8_t pwmChannel = 0,
                              const char *prefsName = nullptr,
                              int8_t loadFeedbackSign = 1,
                              float speedFeedbackBlend = 1.0f,
                              float loadFeedbackBlend = 0.25f,
                              float velocityFeedForwardLimit = 0.58f)
      : _servo(servo), _motor(pwmChannel), _prefsName(prefsName),
        _loadFeedbackSign(loadFeedbackSign),
        _speedFeedbackBlend(constrain(speedFeedbackBlend, 0.05f, 1.0f)),
        _loadFeedbackBlend(constrain(loadFeedbackBlend, 0.05f, 1.0f)),
        _velocityFeedForwardLimit(constrain(velocityFeedForwardLimit, 0.35f, 0.85f)) {}

  void begin() {
    _servo.setExternalDynamic(true);
    _motor.begin();
    _encoder.begin();
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_MOTOR_CURRENT, ADC_11db);
    pinMode(PIN_MOTOR_CURRENT, INPUT);

    if (_encoder.online()) {
      _encoder.update();
      restoreMultiTurnPosition();
      _positionTarget4096 = clampPositionCommand(_encoder.position4096());
      resetPositionProfile();
      _lastGoalRaw = encodePositionCommand(_positionTarget4096);
      _positionTargetValid = true;
      _servo.setRegU16(HLS_REG_GOAL_POSITION_L, _lastGoalRaw);
      saveMultiTurnPosition(true);
    }
    updateFeedback(false);
  }

  void update() {
    const uint32_t now = micros();
    if (now - _lastUpdateUs < 1000) return;
    const float dt = _lastUpdateUs == 0 ? 0.001f : (now - _lastUpdateUs) / 1000000.0f;
    _lastUpdateUs = now;

    const bool encoderOk = _encoder.update();
    _currentMa = readCurrentMa();
    updateLimiterCurrentFilter();
    updateFaults(encoderOk);

    const bool torqueOn = _servo.torqueEnabled();
    if (!torqueOn || _errorFlags != 0) {
      _motor.coast();
      _positionIntegral = 0.0f;
      _speedIntegral = 0.0f;
      _currentIntegral = 0.0f;
      _currentLimitScale = 0.0f;
      _currentLimitFilteredMa = 0.0f;
      _profileValid = false;
      updateFeedback(false);
      saveMultiTurnPosition(_wasTorqueOn && !torqueOn);
      _wasTorqueOn = torqueOn;
      return;
    }

    if (!_wasTorqueOn) {
      _positionIntegral = 0.0f;
      _speedIntegral = 0.0f;
      _currentIntegral = 0.0f;
      _currentLimitScale = 0.0f;
      resetPositionProfile();
    }
    _wasTorqueOn = true;

    float effort = 0.0f;
    switch (_servo.mode()) {
      case 1:
        effort = speedControl(dt);
        break;
      case 2:
        effort = currentControl(dt);
        break;
      case 3:
      case 4:
        effort = pwmControl();
        break;
      case 0:
      default:
        effort = positionControl(dt);
        break;
    }

    if ((_servo.reg(HLS_REG_SETTING) & 0x01) != 0) {
      effort = -effort;
    }
    const float limitedEffort = applyCurrentLimit(effort);
    _motor.drive(limitedEffort);
    const bool moving = fabsf(limitedEffort) > 0.02f;
    updateFeedback(moving);
    saveMultiTurnPosition(!moving && _lastMovingForSave);
    _lastMovingForSave = moving;
  }

  bool encoderOnline() const { return _encoder.online(); }
  bool motorFaulted() const { return _motor.faulted(); }
  int16_t currentMa() const { return _currentMa; }
  uint8_t errorFlags() const { return _errorFlags; }

private:
  HLS3606Emu &_servo;
  DRV8876Motor _motor;
  MT6701Encoder _encoder;
  const char *_prefsName;
  int8_t _loadFeedbackSign;
  float _speedFeedbackBlend;
  float _loadFeedbackBlend;
  float _velocityFeedForwardLimit;
  uint32_t _lastUpdateUs = 0;
  bool _wasTorqueOn = false;
  bool _lastMovingForSave = false;
  bool _positionTargetValid = false;
  float _positionIntegral = 0.0f;
  float _speedIntegral = 0.0f;
  float _currentIntegral = 0.0f;
  bool _profileValid = false;
  float _profilePosition4096 = 0.0f;
  float _profileVelocity4096 = 0.0f;
  float _speedFeedback = 0.0f;
  float _loadFeedback = 0.0f;
  float _currentLimitScale = 0.0f;
  float _currentLimitFilteredMa = 0.0f;
  int32_t _positionTarget4096 = 0;
  int32_t _lastSavedPosition4096 = INT32_MIN;
  int16_t _lastDriveCurrentLimit = -1;
  uint16_t _lastGoalRaw = 0;
  uint32_t _lastPositionSaveMs = 0;
  int16_t _currentMa = 0;
  uint8_t _errorFlags = 0;

  int16_t readCurrentMa() {
    uint32_t totalMv = 0;
    for (uint8_t i = 0; i < 4; ++i) {
      totalMv += analogReadMilliVolts(PIN_MOTOR_CURRENT);
    }
    const float mv = static_cast<float>(totalMv) / 4.0f;
    const int16_t ma = static_cast<int16_t>(mv * CURRENT_MA_PER_MV);
    return clampValue<int16_t>(ma, 0, 2000);
  }

  void updateLimiterCurrentFilter() {
    if (_currentLimitFilteredMa <= 0.01f) {
      _currentLimitFilteredMa = static_cast<float>(_currentMa);
      return;
    }
    const float alpha = static_cast<float>(_currentMa) > _currentLimitFilteredMa ? 0.42f : 0.16f;
    _currentLimitFilteredMa =
        (_currentLimitFilteredMa * (1.0f - alpha)) + (static_cast<float>(_currentMa) * alpha);
  }

  void updateFaults(bool encoderOk) {
    _errorFlags = 0;
    if (!encoderOk) _errorFlags |= HLS_ERROR_ENCODER;
    if (_motor.faulted()) _errorFlags |= HLS_ERROR_FAULT;
    if (_currentMa > hardCurrentLimitMa() + 120) _errorFlags |= HLS_ERROR_OVERCURRENT;
  }

  int16_t hardCurrentLimitMa() const {
    const uint16_t configured = _servo.regU16(HLS_REG_PROTECT_CURRENT_L);
    if (configured == 0) return HARDWARE_CURRENT_LIMIT_MA;
    return min<int16_t>(HARDWARE_CURRENT_LIMIT_MA, configured);
  }

  int16_t driveCurrentLimitMa() const {
    int16_t limit = hardCurrentLimitMa();
    if (_servo.mode() == 0) {
      const uint16_t command = _servo.regU16(HLS_REG_GOAL_CURRENT_L) & 0x7FFF;
      limit = min<int16_t>(limit, static_cast<int16_t>(min<uint16_t>(command, HARDWARE_CURRENT_LIMIT_MA)));
    }
    return limit;
  }

  float torqueLimitScale() const {
    const uint16_t limit = _servo.regU16(HLS_REG_TORQUE_LIMIT_L);
    if (limit == 0) return 1.0f;
    return constrain(limit / 1000.0f, 0.05f, 1.0f);
  }

  float minimumStartEffort() const {
    return constrain(max(0.025f, _servo.reg(HLS_REG_MIN_PWM_L) / 1000.0f), 0.025f, 0.12f);
  }

  static int16_t shortestError4096(uint16_t target, uint16_t present) {
    int16_t err = static_cast<int16_t>(target) - static_cast<int16_t>(present);
    if (err > 2048) err -= 4096;
    if (err < -2048) err += 4096;
    return err;
  }

  static int32_t positiveModulo4096(int32_t value) {
    int32_t result = value % 4096;
    if (result < 0) result += 4096;
    return result;
  }

  static int32_t clampPositionCommand(int32_t position) {
    return clampValue<int32_t>(position, HLS_MIN_POSITION_COMMAND, HLS_MAX_POSITION_COMMAND);
  }

  static int32_t decodePositionCommand(uint16_t raw) {
    const int32_t magnitude = static_cast<int32_t>(raw & 0x7FFF);
    return (raw & 0x8000) != 0 ? -magnitude : magnitude;
  }

  static uint16_t encodePositionCommand(int32_t position) {
    const int32_t clipped = clampPositionCommand(position);
    const uint16_t magnitude = static_cast<uint16_t>(labs(clipped));
    return clipped < 0 ? static_cast<uint16_t>(0x8000 | magnitude) : magnitude;
  }

  void updatePositionTargetFromGoal() {
    if (!_encoder.online()) return;
    const uint16_t goalRaw = _servo.regU16(HLS_REG_GOAL_POSITION_L);
    if (_positionTargetValid && goalRaw == _lastGoalRaw) return;

    _positionTarget4096 = clampPositionCommand(decodePositionCommand(goalRaw));
    _lastGoalRaw = encodePositionCommand(_positionTarget4096);
    if (goalRaw != _lastGoalRaw) {
      _servo.setRegU16(HLS_REG_GOAL_POSITION_L, _lastGoalRaw);
    }
    _positionTargetValid = true;
    resetPositionProfile();
    _positionIntegral = 0.0f;
    _speedIntegral = 0.0f;
  }

  float positionControl(float dt) {
    updatePositionTargetFromGoal();
    if (!_profileValid) resetPositionProfile();

    const float target = static_cast<float>(_positionTarget4096);
    const float present = static_cast<float>(_encoder.position4096());
    const float profileError = target - _profilePosition4096;
    const float presentError = target - present;
    if (fabsf(presentError) <= 4.0f && fabsf(_profileVelocity4096) < 12.0f) {
      _profilePosition4096 = target;
      _profileVelocity4096 = 0.0f;
      _positionIntegral = 0.0f;
      _speedIntegral = 0.0f;
      return 0.0f;
    }

    const float speedLimit = positionSpeedLimit();
    const float accelLimit = positionAccelLimit();
    if (speedLimit < 1.0f || accelLimit < 1.0f) {
      _profilePosition4096 = present;
      _profileVelocity4096 = 0.0f;
      _speedIntegral = 0.0f;
      return 0.0f;
    }
    const float brakingSpeed = sqrtf(max(0.0f, 2.0f * accelLimit * fabsf(profileError)));
    float desiredVelocity = 0.0f;
    if (fabsf(profileError) > 0.8f) {
      desiredVelocity = (profileError > 0.0f ? 1.0f : -1.0f) *
                        min(speedLimit, brakingSpeed);
    }

    const float velocityDeltaLimit = accelLimit * dt;
    _profileVelocity4096 += constrain(desiredVelocity - _profileVelocity4096,
                                      -velocityDeltaLimit, velocityDeltaLimit);
    _profilePosition4096 += _profileVelocity4096 * dt;
    if ((target - _profilePosition4096) * profileError < 0.0f) {
      _profilePosition4096 = target;
      _profileVelocity4096 = 0.0f;
    }

    const float trackingError = _profilePosition4096 - present;
    const float trackingKp = constrain(_servo.reg(HLS_REG_P) / 5.5f, 3.0f, 8.0f);
    float speedTarget = _profileVelocity4096 + (trackingError * trackingKp);
    const float arrivalWindow = constrain(speedLimit * 0.32f, 500.0f, 1800.0f);
    const float arrivalScale =
        1.0f - constrain(fabsf(presentError) / arrivalWindow, 0.0f, 1.0f);
    const float dampingGain = constrain(_servo.reg(HLS_REG_D) / 22.0f, 0.0f, 0.55f);
    speedTarget -= static_cast<float>(_encoder.speed4096PerSec()) * dampingGain * arrivalScale;
    speedTarget = constrain(speedTarget, -speedLimit, speedLimit);

    float effort = velocityEffort(speedTarget, dt);
    effort = applyVelocityLimit(effort, speedLimit);
    const float minEffort = minimumStartEffort();
    if (fabsf(speedTarget) > 25.0f && fabsf(effort) < minEffort) {
      effort = speedTarget > 0.0f ? minEffort : -minEffort;
    }
    effort = applyVelocityLimit(effort, speedLimit);
    return effort * torqueLimitScale();
  }

  float speedControl(float dt) {
    int16_t target = _servo.regSignedMagnitude15(HLS_REG_RUN_SPEED_L);
    if (target == 0) return 0.0f;
    target = clampValue<int16_t>(target, -3000, 3000);
    const float speedLimit = fabsf(static_cast<float>(target) * FEETECH_SPEED_COUNTS_PER_UNIT);
    float effort = velocityEffort(static_cast<float>(target) * FEETECH_SPEED_COUNTS_PER_UNIT, dt);
    effort = applyVelocityLimit(effort, speedLimit);
    return effort * torqueLimitScale();
  }

  float velocityEffort(float target, float dt) {
    target = constrain(target, -30000.0f, 30000.0f);
    const int16_t speed = _encoder.speed4096PerSec();
    const float error = target - static_cast<float>(speed);
    const float kp = max(0.018f, _servo.reg(HLS_REG_VELOCITY_P) / 420.0f);
    const float ki = max(0.00003f, _servo.reg(HLS_REG_VELOCITY_I) / 18000.0f);
    _speedIntegral = constrain(_speedIntegral + error * dt, -9000.0f, 9000.0f);
    const float feedForward = constrain(target / 12000.0f,
                                        -_velocityFeedForwardLimit,
                                        _velocityFeedForwardLimit);
    float effort = constrain(feedForward + (kp * error / 4096.0f) + (ki * _speedIntegral),
                             -1.0f, 1.0f);
    const float minEffort = minimumStartEffort();
    if (fabsf(target) > 20.0f && abs(speed) < 80 && fabsf(effort) < minEffort) {
      effort = target > 0.0f ? minEffort : -minEffort;
    }
    return effort;
  }

  float currentControl(float dt) {
    int16_t target = _servo.regSignedMagnitude15(HLS_REG_GOAL_CURRENT_L);
    const int16_t limit = hardCurrentLimitMa();
    target = clampValue<int16_t>(target, static_cast<int16_t>(-limit), limit);
    const int16_t signedCurrent = static_cast<int16_t>(_currentMa * (_motor.lastEffort() < 0 ? -1 : 1));
    const float error = static_cast<float>(target - signedCurrent);
    const float kp = max(0.025f, _servo.reg(HLS_REG_CURRENT_P) / 350.0f);
    const float ki = _servo.reg(HLS_REG_CURRENT_I) / 30000.0f;
    _currentIntegral = constrain(_currentIntegral + error * dt, -limit, limit);
    const float feedForward = constrain(static_cast<float>(target) / limit * 0.18f, -0.45f, 0.45f);
    float effort = constrain(feedForward + (kp * error / limit) + (ki * _currentIntegral / limit),
                             -1.0f, 1.0f);
    const float minEffort = minimumStartEffort();
    if (target != 0 && _currentMa < 20 && abs(_encoder.speed4096PerSec()) < 80 &&
        fabsf(effort) < minEffort) {
      effort = target > 0 ? minEffort : -minEffort;
    }
    return effort * torqueLimitScale();
  }

  float pwmControl() const {
    const int16_t command = _servo.regSignedMagnitude15(HLS_REG_GOAL_CURRENT_L);
    return constrain(command / 1000.0f, -1.0f, 1.0f) * torqueLimitScale();
  }

  float applyVelocityLimit(float effort, float speedLimit4096PerSec) {
    if (speedLimit4096PerSec < 1.0f) {
      _speedIntegral = 0.0f;
      return 0.0f;
    }

    const float speed = static_cast<float>(_encoder.speed4096PerSec());
    const float absSpeed = fabsf(speed);
    const bool acceleratingSameDirection = speed != 0.0f && (effort * speed) > 0.0f;
    if (absSpeed >= speedLimit4096PerSec) {
      const bool braking = speed != 0.0f && (effort * speed) < 0.0f;
      if (braking) {
        return effort;
      }
      _speedIntegral = 0.0f;
      const float overspeed = absSpeed - speedLimit4096PerSec;
      const float brake = constrain(0.025f + (overspeed / max(speedLimit4096PerSec, 1.0f)) * 0.20f,
                                    0.025f, 0.16f);
      return speed > 0.0f ? -brake : brake;
    }

    const float guardBand = max(24.0f, speedLimit4096PerSec * 0.08f);
    const float headroom = speedLimit4096PerSec - absSpeed;
    if (acceleratingSameDirection && headroom < guardBand) {
      return effort * constrain(headroom / guardBand, 0.0f, 1.0f);
    }
    return effort;
  }

  float applyCurrentLimit(float effort) {
    const int16_t limit = driveCurrentLimitMa();
    if (limit <= 0) {
      _currentIntegral = 0.0f;
      _speedIntegral = 0.0f;
      _currentLimitScale = 0.0f;
      return 0.0f;
    }

    const float emergencyLimit =
        min(static_cast<float>(HARDWARE_CURRENT_LIMIT_MA),
            static_cast<float>(limit) + max(140.0f, static_cast<float>(limit) * 0.95f));
    if (_currentMa >= emergencyLimit) {
      _currentIntegral = 0.0f;
      _speedIntegral = 0.0f;
      _currentLimitScale = 0.0f;
      return 0.0f;
    }

    if (_lastDriveCurrentLimit != limit) {
      const float startupCap = constrain(static_cast<float>(limit) / 450.0f, 0.03f, 0.35f);
      _currentLimitScale = min(_currentLimitScale, startupCap);
      _lastDriveCurrentLimit = limit;
    }

    const float filteredCurrent = max(_currentLimitFilteredMa, static_cast<float>(_currentMa) * 0.90f);
    const float guardStart = max(4.0f, static_cast<float>(limit) * 0.78f);
    const float softCeiling = static_cast<float>(limit) + max(8.0f, static_cast<float>(limit) * 0.10f);
    if (filteredCurrent > static_cast<float>(limit)) {
      const float overRatio =
          (filteredCurrent - static_cast<float>(limit)) / max(static_cast<float>(limit), 1.0f);
      const float decay = constrain(0.0060f + overRatio * 0.040f, 0.0060f, 0.050f);
      _currentLimitScale = max(0.025f, _currentLimitScale - decay);
      _currentIntegral = 0.0f;
      _speedIntegral *= 0.94f;
    } else if (filteredCurrent > guardStart) {
      const float range = max(1.0f, softCeiling - guardStart);
      const float allowedScale = constrain((softCeiling - filteredCurrent) / range, 0.08f, 1.0f);
      _currentLimitScale = min(_currentLimitScale, allowedScale);
    } else {
      const float step = limit < 80 ? 0.0024f : 0.0038f;
      _currentLimitScale = min(1.0f, _currentLimitScale + step);
    }

    return constrain(effort, -_currentLimitScale, _currentLimitScale);
  }

  float positionSpeedLimit() const {
    const uint16_t command = _servo.regU16(HLS_REG_RUN_SPEED_L) & 0x7FFF;
    if (command == 0) return 0.0f;
    const float limit = static_cast<float>(command) * FEETECH_SPEED_COUNTS_PER_UNIT;
    return constrain(limit, 0.0f, 30000.0f);
  }

  float positionAccelLimit() const {
    const uint8_t command = _servo.reg(HLS_REG_GOAL_ACCEL);
    const float accel = static_cast<float>(command) * 110.0f;
    return constrain(accel, 0.0f, 26000.0f);
  }

  void resetPositionProfile() {
    if (!_encoder.online()) return;
    _profilePosition4096 = static_cast<float>(_encoder.position4096());
    _profileVelocity4096 = 0.0f;
    _profileValid = true;
  }

  uint16_t displayPosition4096() const {
    if (!_encoder.online()) return _servo.regU16(HLS_REG_PRESENT_POSITION_L);
    return encodePositionCommand(_encoder.position4096());
  }

  void updateFeedback(bool moving) {
    int16_t rawSpeed = _encoder.online() ? _encoder.speed4096PerSec() : 0;
    uint16_t position = displayPosition4096();
    if (_encoder.online() && _positionTargetValid &&
        labs(_positionTarget4096 - _encoder.position4096()) <= 8 &&
        fabsf(_motor.lastEffort()) < 0.03f) {
      position = encodePositionCommand(_positionTarget4096);
      rawSpeed = 0;
    }
    const int16_t measuredSpeed =
        static_cast<int16_t>(lroundf(static_cast<float>(rawSpeed) / FEETECH_SPEED_COUNTS_PER_UNIT));
    if (!moving || measuredSpeed == 0) {
      _speedFeedback = 0.0f;
    } else if ((_speedFeedback > 0.0f && measuredSpeed < 0) ||
               (_speedFeedback < 0.0f && measuredSpeed > 0)) {
      _speedFeedback = static_cast<float>(measuredSpeed);
    } else {
      _speedFeedback = (_speedFeedback * (1.0f - _speedFeedbackBlend)) +
                       (static_cast<float>(measuredSpeed) * _speedFeedbackBlend);
    }
    const int16_t speed = static_cast<int16_t>(lroundf(_speedFeedback));

    const float targetLoad = _motor.lastEffort() * 850.0f * static_cast<float>(_loadFeedbackSign);
    _loadFeedback = (_loadFeedback * (1.0f - _loadFeedbackBlend)) +
                    (targetLoad * _loadFeedbackBlend);
    const int16_t load = static_cast<int16_t>(constrain(_loadFeedback, -1000.0f, 1000.0f));

    _servo.setHardwareFeedback(position, speed, load, 60, 25, moving ? 1 : 0,
                               static_cast<int16_t>(_currentMa), _errorFlags);
    _servo.setRegU16(HLS_DIAG_RAW14_L, _encoder.lastRaw14());
    setRegI32(HLS_DIAG_MULTI_POS_0, _encoder.position4096());
    _servo.setReg(HLS_DIAG_FIRST_I2C, _encoder.firstAddress());
    _servo.setReg(HLS_DIAG_ACTIVE_I2C, _encoder.activeAddress());
    _servo.setReg(HLS_DIAG_ENCODER_STATUS, _encoder.lastStatus());
  }

  void setRegI32(uint8_t addr, int32_t value) {
    _servo.setReg(addr, static_cast<uint8_t>(value & 0xFF));
    _servo.setReg(addr + 1, static_cast<uint8_t>((value >> 8) & 0xFF));
    _servo.setReg(addr + 2, static_cast<uint8_t>((value >> 16) & 0xFF));
    _servo.setReg(addr + 3, static_cast<uint8_t>((value >> 24) & 0xFF));
  }

  void restoreMultiTurnPosition() {
    if (!_prefsName) return;
    Preferences prefs;
    prefs.begin(_prefsName, false);
    const int32_t saved = prefs.getInt("pos4096", INT32_MIN);
    prefs.end();
    if (saved == INT32_MIN) return;
    _encoder.alignToSavedPosition4096(saved);
    _lastSavedPosition4096 = _encoder.position4096();
  }

  void saveMultiTurnPosition(bool force) {
    if (!_prefsName || !_encoder.online()) return;
    const uint32_t now = millis();
    const int32_t position = _encoder.position4096();
    if (!force && now - _lastPositionSaveMs < 2000) return;
    if (!force && _lastSavedPosition4096 != INT32_MIN &&
        labs(position - _lastSavedPosition4096) < 16) {
      return;
    }

    Preferences prefs;
    prefs.begin(_prefsName, false);
    prefs.putInt("pos4096", position);
    prefs.end();
    _lastSavedPosition4096 = position;
    _lastPositionSaveMs = now;
  }
};

#endif
