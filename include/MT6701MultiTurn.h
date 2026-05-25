#ifndef MT6701MultiTurn_h
#define MT6701MultiTurn_h

#include <Arduino.h>
#include <Wire.h>

class MT6701MultiTurn {
private:
  uint8_t _addr;
  uint8_t _sda;
  uint8_t _scl;

  float _lastAngle;
  int32_t _totalTurns;

  // 读取单圈原始角度 0~360°
  float readSingleAngle() {
    Wire.beginTransmission(_addr);
    Wire.write(0x03);
    if (Wire.endTransmission(true) != 0) {
      return _lastAngle;
    }

    Wire.requestFrom(_addr, (uint8_t)2);
    if (Wire.available() < 2) {
      return _lastAngle;
    }

    uint16_t raw = (Wire.read() << 8) | Wire.read();
    raw >>= 2;
    float angle = raw * 360.0f / 16383.0f;
    return angle;
  }

public:
  // 构造函数：I2C地址、SDA、SCL
  MT6701MultiTurn(uint8_t addr = 0x06, uint8_t sda = 8, uint8_t scl = 9) {
    _addr = addr;
    _sda = sda;
    _scl = scl;
    _totalTurns = 0;
  }

  // 初始化 I2C
  void begin() {
    Wire.begin(_sda, _scl);
    _lastAngle = readSingleAngle();
  }

  // 读取多圈累计角度（可正可负，无上限）
  float readAngle() {
    float current = readSingleAngle();
    float diff = current - _lastAngle;

    if (diff > 355.0f) {
      _totalTurns--;
    } else if (diff < -355.0f) {
      _totalTurns++;
    }

    _lastAngle = current;
    return _totalTurns * 360.0f + current;
  }

  // 读取单圈 0~360°
  float readSingle() {
    return readSingleAngle();
  }

  // 获取当前圈数
  int32_t getTurns() {
    return _totalTurns;
  }

  // 重置多圈计数（归零）
  void reset() {
    _totalTurns = 0;
    _lastAngle = readSingleAngle();
  }
};

#endif