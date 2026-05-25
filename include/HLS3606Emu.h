#ifndef HLS3606_EMU_H
#define HLS3606_EMU_H

#include <Arduino.h>
#include <Preferences.h>

static constexpr uint8_t HLS_INST_PING = 0x01;
static constexpr uint8_t HLS_INST_READ = 0x02;
static constexpr uint8_t HLS_INST_WRITE = 0x03;
static constexpr uint8_t HLS_INST_REG_WRITE = 0x04;
static constexpr uint8_t HLS_INST_ACTION = 0x05;
static constexpr uint8_t HLS_INST_RESET = 0x06;
static constexpr uint8_t HLS_INST_SYNC_WRITE = 0x83;
static constexpr uint8_t HLS_BROADCAST_ID = 0xFE;

static constexpr uint8_t HLS_REG_FIRMWARE_MAIN = 0x00;
static constexpr uint8_t HLS_REG_FIRMWARE_SUB = 0x01;
static constexpr uint8_t HLS_REG_MODEL_L = 0x03;
static constexpr uint8_t HLS_REG_MODEL_H = 0x04;
static constexpr uint8_t HLS_REG_ID = 0x05;
static constexpr uint8_t HLS_REG_BAUD = 0x06;
static constexpr uint8_t HLS_REG_RETURN_DELAY = 0x07;
static constexpr uint8_t HLS_REG_STATUS_LEVEL = 0x08;
static constexpr uint8_t HLS_REG_MIN_ANGLE_L = 0x09;
static constexpr uint8_t HLS_REG_MAX_ANGLE_L = 0x0B;
static constexpr uint8_t HLS_REG_MAX_TEMP = 0x0D;
static constexpr uint8_t HLS_REG_MAX_VOLT = 0x0E;
static constexpr uint8_t HLS_REG_MIN_VOLT = 0x0F;
static constexpr uint8_t HLS_REG_MAX_TORQUE_L = 0x10;
static constexpr uint8_t HLS_REG_P = 0x15;
static constexpr uint8_t HLS_REG_D = 0x16;
static constexpr uint8_t HLS_REG_I = 0x17;
static constexpr uint8_t HLS_REG_MIN_PWM_L = 0x18;
static constexpr uint8_t HLS_REG_OFFSET_L = 0x21;
static constexpr uint8_t HLS_REG_MODE = 0x23;
static constexpr uint8_t HLS_REG_PROTECT_CURRENT_L = 0x24;
static constexpr uint8_t HLS_REG_TORQUE_ENABLE = 0x28;
static constexpr uint8_t HLS_REG_GOAL_POSITION_L = 0x2A;
static constexpr uint8_t HLS_REG_RUN_TIME_L = 0x2C;
static constexpr uint8_t HLS_REG_RUN_SPEED_L = 0x2E;
static constexpr uint8_t HLS_REG_LOCK = 0x30;
static constexpr uint8_t HLS_REG_PRESENT_POSITION_L = 0x38;
static constexpr uint8_t HLS_REG_PRESENT_SPEED_L = 0x3A;
static constexpr uint8_t HLS_REG_PRESENT_LOAD_L = 0x3C;
static constexpr uint8_t HLS_REG_PRESENT_VOLTAGE = 0x3E;
static constexpr uint8_t HLS_REG_PRESENT_TEMP = 0x3F;
static constexpr uint8_t HLS_REG_REG_WRITE_FLAG = 0x40;
static constexpr uint8_t HLS_REG_ERROR = 0x41;
static constexpr uint8_t HLS_REG_MOVING = 0x42;
static constexpr uint8_t HLS_REG_CURRENT_TARGET_L = 0x43;
static constexpr uint8_t HLS_REG_PRESENT_CURRENT_L = 0x45;

static constexpr size_t HLS_TABLE_SIZE = 96;
static constexpr size_t HLS_DATA_MAX = 160;

typedef void (*HlsPacketWriter)(const uint8_t *data, size_t len);

class HLS3606Emu {
public:
  HLS3606Emu(uint8_t defaultId, uint8_t defaultBaudCode, const char *prefsName)
      : _defaultId(defaultId), _defaultBaudCode(defaultBaudCode), _prefsName(prefsName) {}

  void begin() {
    setDefaults();
    loadPersistent();
    _targetPosition = clampPosition(readU16(HLS_REG_GOAL_POSITION_L));
    _presentPosition = _targetPosition;
    _lastMotionMs = millis();
    updateDynamic();
  }

  uint8_t id() const { return _table[HLS_REG_ID]; }
  uint8_t baudCode() const { return _table[HLS_REG_BAUD]; }
  uint32_t goodPackets() const { return _goodPackets; }
  uint32_t badPackets() const { return _badPackets; }
  uint32_t statusPackets() const { return _statusPackets; }

  static uint8_t checksum(const uint8_t *buf, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i) sum += buf[i];
    return static_cast<uint8_t>(~sum);
  }

  static uint32_t baudFromCode(uint8_t code) {
    switch (code) {
      case 0: return 1000000;
      case 1: return 500000;
      case 2: return 250000;
      case 3: return 128000;
      case 4: return 115200;
      case 5: return 76800;
      case 6: return 57600;
      case 7: return 38400;
      default: return 115200;
    }
  }

  bool accepts(uint8_t packetId) const {
    return packetId == _table[HLS_REG_ID] || packetId == HLS_BROADCAST_ID;
  }

  void processInstruction(uint8_t packetId, uint8_t inst, const uint8_t *params,
                          uint8_t paramLen, HlsPacketWriter writer) {
    const bool broadcast = packetId == HLS_BROADCAST_ID;
    if (!accepts(packetId)) return;

    ++_goodPackets;
    updateDynamic();

    switch (inst) {
      case HLS_INST_PING:
        if (shouldReturnStatus(inst, broadcast)) sendStatus(writer, _table[HLS_REG_ERROR], nullptr, 0);
        break;
      case HLS_INST_READ:
        handleRead(params, paramLen, broadcast, writer);
        break;
      case HLS_INST_WRITE:
      case HLS_INST_REG_WRITE:
        handleWriteLike(inst, params, paramLen, broadcast, writer);
        break;
      case HLS_INST_ACTION:
        handleAction(broadcast, writer);
        break;
      case HLS_INST_RESET:
        setDefaults();
        savePersistent();
        if (shouldReturnStatus(inst, broadcast)) sendStatus(writer, _table[HLS_REG_ERROR], nullptr, 0);
        break;
      case HLS_INST_SYNC_WRITE:
        handleSyncWrite(params, paramLen);
        break;
      default:
        if (shouldReturnStatus(inst, broadcast)) sendStatus(writer, _table[HLS_REG_ERROR], nullptr, 0);
        break;
    }
  }

private:
  uint8_t _defaultId;
  uint8_t _defaultBaudCode;
  const char *_prefsName;
  uint8_t _table[HLS_TABLE_SIZE] = {};
  uint8_t _regWriteData[64] = {};
  uint8_t _regWriteAddr = 0;
  uint8_t _regWriteLen = 0;
  bool _regWritePending = false;
  uint16_t _presentPosition = 2048;
  uint16_t _targetPosition = 2048;
  uint16_t _presentSpeed = 0;
  uint16_t _presentLoad = 0;
  uint16_t _presentCurrent = 0;
  uint32_t _lastMotionMs = 0;
  uint32_t _goodPackets = 0;
  uint32_t _badPackets = 0;
  uint32_t _statusPackets = 0;
  Preferences _prefs;

  uint16_t readU16(uint8_t addr) const {
    if (addr + 1 >= HLS_TABLE_SIZE) return 0;
    return static_cast<uint16_t>(_table[addr]) |
           (static_cast<uint16_t>(_table[addr + 1]) << 8);
  }

  void writeU16(uint8_t addr, uint16_t value) {
    if (addr + 1 >= HLS_TABLE_SIZE) return;
    _table[addr] = static_cast<uint8_t>(value & 0xFF);
    _table[addr + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  }

  uint16_t clampPosition(uint16_t pos) const {
    const uint16_t minPos = readU16(HLS_REG_MIN_ANGLE_L);
    const uint16_t maxPos = readU16(HLS_REG_MAX_ANGLE_L);
    if (minPos < maxPos) {
      if (pos < minPos) return minPos;
      if (pos > maxPos) return maxPos;
    }
    return pos > 4095 ? 4095 : pos;
  }

  void setDefaults() {
    memset(_table, 0, sizeof(_table));
    _table[HLS_REG_FIRMWARE_MAIN] = 3;   // HLS firmware family
    _table[HLS_REG_FIRMWARE_SUB] = 40;   // HLS memory map in FD setup.log
    _table[HLS_REG_MODEL_L] = 10;        // HLS model family
    _table[HLS_REG_MODEL_H] = 10;        // HLS3606 in FD setup.log
    _table[HLS_REG_ID] = _defaultId;
    _table[HLS_REG_BAUD] = _defaultBaudCode;
    _table[HLS_REG_RETURN_DELAY] = 0;
    _table[HLS_REG_STATUS_LEVEL] = 1;
    writeU16(HLS_REG_MIN_ANGLE_L, 0);
    writeU16(HLS_REG_MAX_ANGLE_L, 4095);
    _table[HLS_REG_MAX_TEMP] = 80;
    _table[HLS_REG_MAX_VOLT] = 110;
    _table[HLS_REG_MIN_VOLT] = 40;
    writeU16(HLS_REG_MAX_TORQUE_L, 1000);
    _table[HLS_REG_P] = 15;
    _table[HLS_REG_D] = 0;
    _table[HLS_REG_I] = 0;
    writeU16(HLS_REG_MIN_PWM_L, 100);
    writeU16(HLS_REG_OFFSET_L, 0);
    _table[HLS_REG_MODE] = 0;
    writeU16(HLS_REG_PROTECT_CURRENT_L, 1000);
    _table[HLS_REG_TORQUE_ENABLE] = 0;
    writeU16(HLS_REG_GOAL_POSITION_L, 2048);
    writeU16(HLS_REG_RUN_TIME_L, 0);
    writeU16(HLS_REG_RUN_SPEED_L, 0);
    _table[HLS_REG_LOCK] = 0;
    _table[HLS_REG_PRESENT_VOLTAGE] = 60;
    _table[HLS_REG_PRESENT_TEMP] = 25;
    _table[HLS_REG_ERROR] = 0;
    _table[HLS_REG_MOVING] = 0;
    writeU16(HLS_REG_CURRENT_TARGET_L, 2048);
    writeU16(HLS_REG_PRESENT_CURRENT_L, 0);
  }

  void loadPersistent() {
    _prefs.begin(_prefsName, false);
    _table[HLS_REG_ID] = _prefs.getUChar("id", _table[HLS_REG_ID]);
    _table[HLS_REG_BAUD] = _prefs.getUChar("baud", _table[HLS_REG_BAUD]);
    _table[HLS_REG_RETURN_DELAY] = _prefs.getUChar("ret", _table[HLS_REG_RETURN_DELAY]);
    _table[HLS_REG_STATUS_LEVEL] = _prefs.getUChar("stat", _table[HLS_REG_STATUS_LEVEL]);
    _table[HLS_REG_MODE] = _prefs.getUChar("mode", _table[HLS_REG_MODE]);
    writeU16(HLS_REG_MIN_ANGLE_L, _prefs.getUShort("minp", readU16(HLS_REG_MIN_ANGLE_L)));
    writeU16(HLS_REG_MAX_ANGLE_L, _prefs.getUShort("maxp", readU16(HLS_REG_MAX_ANGLE_L)));
    _prefs.end();
  }

  void savePersistent() {
    _prefs.begin(_prefsName, false);
    _prefs.putUChar("id", _table[HLS_REG_ID]);
    _prefs.putUChar("baud", _table[HLS_REG_BAUD]);
    _prefs.putUChar("ret", _table[HLS_REG_RETURN_DELAY]);
    _prefs.putUChar("stat", _table[HLS_REG_STATUS_LEVEL]);
    _prefs.putUChar("mode", _table[HLS_REG_MODE]);
    _prefs.putUShort("minp", readU16(HLS_REG_MIN_ANGLE_L));
    _prefs.putUShort("maxp", readU16(HLS_REG_MAX_ANGLE_L));
    _prefs.end();
  }

  bool shouldReturnStatus(uint8_t inst, bool broadcast) const {
    if (broadcast) return inst == HLS_INST_PING;
    if (inst == HLS_INST_PING || inst == HLS_INST_READ) return true;
    return _table[HLS_REG_STATUS_LEVEL] != 0;
  }

  void updateDynamic() {
    const bool torqueOn = _table[HLS_REG_TORQUE_ENABLE] != 0;
    const uint32_t now = millis();
    const uint32_t dt = now - _lastMotionMs;
    if (dt == 0) return;
    _lastMotionMs = now;

    if (torqueOn) {
      _targetPosition = clampPosition(readU16(HLS_REG_GOAL_POSITION_L));
      const int32_t diff = static_cast<int32_t>(_targetPosition) - _presentPosition;
      if (diff == 0) {
        _presentSpeed = 0;
        _presentLoad = 0;
        _presentCurrent = 20;
        _table[HLS_REG_MOVING] = 0;
      } else {
        uint32_t speed = readU16(HLS_REG_RUN_SPEED_L) & 0x7FFF;
        if (speed == 0) speed = 4096;
        const uint32_t step = max<uint32_t>(1, (speed * dt) / 1000UL);
        if (step >= static_cast<uint32_t>(abs(diff))) {
          _presentPosition = _targetPosition;
        } else if (diff > 0) {
          _presentPosition += step;
        } else {
          _presentPosition -= step;
        }
        _presentSpeed = min<uint32_t>(speed, 0x7FFF);
        _presentLoad = 120;
        _presentCurrent = 180;
        _table[HLS_REG_MOVING] = _presentPosition == _targetPosition ? 0 : 1;
      }
    } else {
      _presentSpeed = 0;
      _presentLoad = 0;
      _presentCurrent = 0;
      _table[HLS_REG_MOVING] = 0;
    }

    writeU16(HLS_REG_PRESENT_POSITION_L, _presentPosition);
    writeU16(HLS_REG_PRESENT_SPEED_L, _presentSpeed);
    writeU16(HLS_REG_PRESENT_LOAD_L, _presentLoad);
    writeU16(HLS_REG_CURRENT_TARGET_L, _targetPosition);
    writeU16(HLS_REG_PRESENT_CURRENT_L, _presentCurrent);
  }

  void sendStatus(HlsPacketWriter writer, uint8_t err, const uint8_t *params, uint8_t paramLen) {
    if (!writer) return;
    delayMicroseconds(static_cast<uint16_t>(_table[HLS_REG_RETURN_DELAY]) * 2U);
    uint8_t packet[HLS_DATA_MAX + 6];
    const uint8_t len = paramLen + 2;
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = _table[HLS_REG_ID];
    packet[3] = len;
    packet[4] = err;
    for (uint8_t i = 0; i < paramLen; ++i) packet[5 + i] = params[i];
    packet[5 + paramLen] = checksum(&packet[2], static_cast<size_t>(len + 1));
    writer(packet, static_cast<size_t>(6 + paramLen));
    ++_statusPackets;
  }

  void handleRead(const uint8_t *params, uint8_t paramLen, bool broadcast, HlsPacketWriter writer) {
    if (paramLen < 2) {
      if (!broadcast) sendStatus(writer, _table[HLS_REG_ERROR], nullptr, 0);
      return;
    }
    updateDynamic();
    const uint8_t addr = params[0];
    uint8_t len = params[1];
    if (len > HLS_DATA_MAX) len = HLS_DATA_MAX;
    uint8_t out[HLS_DATA_MAX];
    for (uint8_t i = 0; i < len; ++i) {
      const uint8_t a = addr + i;
      out[i] = a < HLS_TABLE_SIZE ? _table[a] : 0;
    }
    if (!broadcast) sendStatus(writer, _table[HLS_REG_ERROR], out, len);
  }

  void applyWrite(uint8_t addr, const uint8_t *data, uint8_t len) {
    for (uint8_t i = 0; i < len; ++i) {
      const uint8_t a = addr + i;
      if (a >= HLS_TABLE_SIZE) break;
      _table[a] = data[i];
    }
    if (_table[HLS_REG_ID] > 253) _table[HLS_REG_ID] = _defaultId;
    if (_table[HLS_REG_BAUD] > 7) _table[HLS_REG_BAUD] = _defaultBaudCode;
    _targetPosition = clampPosition(readU16(HLS_REG_GOAL_POSITION_L));
    writeU16(HLS_REG_GOAL_POSITION_L, _targetPosition);
    writeU16(HLS_REG_CURRENT_TARGET_L, _targetPosition);
    savePersistent();
  }

  void handleWriteLike(uint8_t inst, const uint8_t *params, uint8_t paramLen,
                       bool broadcast, HlsPacketWriter writer) {
    if (paramLen < 1) {
      if (shouldReturnStatus(inst, broadcast)) sendStatus(writer, _table[HLS_REG_ERROR], nullptr, 0);
      return;
    }
    const uint8_t addr = params[0];
    const uint8_t dataLen = paramLen - 1;
    if (inst == HLS_INST_REG_WRITE) {
      _regWriteAddr = addr;
      _regWriteLen = min<uint8_t>(dataLen, sizeof(_regWriteData));
      memcpy(_regWriteData, params + 1, _regWriteLen);
      _regWritePending = true;
      _table[HLS_REG_REG_WRITE_FLAG] = 1;
    } else {
      applyWrite(addr, params + 1, dataLen);
    }
    if (shouldReturnStatus(inst, broadcast)) sendStatus(writer, _table[HLS_REG_ERROR], nullptr, 0);
  }

  void handleAction(bool broadcast, HlsPacketWriter writer) {
    if (_regWritePending) {
      applyWrite(_regWriteAddr, _regWriteData, _regWriteLen);
      _regWritePending = false;
      _table[HLS_REG_REG_WRITE_FLAG] = 0;
    }
    if (shouldReturnStatus(HLS_INST_ACTION, broadcast)) sendStatus(writer, _table[HLS_REG_ERROR], nullptr, 0);
  }

  void handleSyncWrite(const uint8_t *params, uint8_t paramLen) {
    if (paramLen < 2) return;
    const uint8_t addr = params[0];
    const uint8_t dataLen = params[1];
    uint8_t index = 2;
    while (index + 1 + dataLen <= paramLen) {
      const uint8_t id = params[index++];
      if (id == _table[HLS_REG_ID] || id == HLS_BROADCAST_ID) {
        applyWrite(addr, params + index, dataLen);
      }
      index += dataLen;
    }
  }
};

#endif
