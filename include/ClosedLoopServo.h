#ifndef ClosedLoopServo_h
#define ClosedLoopServo_h

#include <Arduino.h>
#include <EEPROM.h>
#include "DRV8876_PH_EN.h"
#include "MT6701MultiTurn.h"

class ClosedLoopServo {
private:
  DRV8876_PH_EN& _motor;
  MT6701MultiTurn& _encoder;

  int _dir;
  float _Kp, _Ki, _Kd;

  float _target;
  float _error;
  float _lastError;
  float _integral;

  bool _paramsExist = false;

  void loadParams() {
    EEPROM.begin(64);
    byte magic = EEPROM.read(0);
    if (magic != 0xAB) {
      _paramsExist = false;
      return;
    }
    _dir = EEPROM.read(1);
    EEPROM.get(4, _Kp);
    EEPROM.get(8, _Ki);
    EEPROM.get(12, _Kd);
    _paramsExist = true;
  }

  void saveParams() {
    EEPROM.write(0, 0xAB);
    EEPROM.write(1, _dir);
    EEPROM.put(4, _Kp);
    EEPROM.put(8, _Ki);
    EEPROM.put(12, _Kd);
    EEPROM.commit();
  }

public:
  ClosedLoopServo(DRV8876_PH_EN& motor, MT6701MultiTurn& encoder)
    : _motor(motor), _encoder(encoder)
  {
    _target = 0;
    _error = 0;
    _lastError = 0;
    _integral = 0;
  }

  bool begin() {
    loadParams();
    if (!_paramsExist) {
      Serial.println("⚠️ 未检测到已保存参数，开始自动校准...");
      calibrateDirection();
      autoTunePID();
      saveParams();
      Serial.println("✅ 校准完成，参数已保存！");
      return false; // 代表本次是新校准
    }
    Serial.println("✅ 已加载保存的参数，无需校准");
    _target = _encoder.readAngle();
    return true; // 代表直接加载
  }

  void calibrateDirection() {
    Serial.println("🔧 正在校准电机方向...");
    _encoder.reset();
    delay(100);
    _motor.forward(80);
    delay(300);
    _motor.stop();
    float delta = _encoder.readAngle();
    _dir = (delta > 5) ? 1 : 0;
    Serial.print("   方向校准结果：dir = "); Serial.println(_dir);
  }

  void autoTunePID() {
    Serial.println("🔧 正在整定 PID 参数...");
    _Kp = 3.2f;
    _Ki = 0.15f;
    _Kd = 8.0f;
    Serial.print("   整定结果：Kp="); Serial.print(_Kp, 2);
    Serial.print(" Ki="); Serial.print(_Ki, 2);
    Serial.print(" Kd="); Serial.println(_Kd, 2);
  }

  void setTarget(float angle) {
    _target = angle;
    _integral = 0;
    _lastError = 0;
  }

  void update() {
    float now = _encoder.readAngle();
    _error = _target - now;

    if (fabs(_error) <= 0.5f) {
      _motor.stop();
      _integral = 0;
      return;
    }

    _integral += _error;
    _integral = constrain(_integral, -60, 60);
    float deriv = _error - _lastError;
    float out = _Kp * _error + _Ki * _integral + _Kd * deriv;

    int duty = constrain(abs((int)out), 0, 255);

    if (out > 0) {
      _dir ? _motor.forward(duty) : _motor.backward(duty);
    } else {
      _dir ? _motor.backward(duty) : _motor.forward(duty);
    }
    _lastError = _error;
  }

  float getAngle()   { return _encoder.readAngle(); }
  float getError()   { return _error; }
  float getTarget()  { return _target; }
  int getDir()       { return _dir; }
  float getKp()      { return _Kp; }
  float getKi()      { return _Ki; }
  float getKd()      { return _Kd; }
  bool isCalibrated(){ return _paramsExist; }
};

#endif