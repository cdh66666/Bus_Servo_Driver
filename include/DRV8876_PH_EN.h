#ifndef DRV8876_PH_EN_h
#define DRV8876_PH_EN_h

#include <Arduino.h>

class DRV8876_PH_EN {
private:
  int _enPin;
  int _phPin;

public:
  // 构造函数：EN PWM引脚, PH 方向引脚
  DRV8876_PH_EN(int enPin, int phPin) {
    _enPin = enPin;
    _phPin = phPin;
    pinMode(_enPin, OUTPUT);
    pinMode(_phPin, OUTPUT);
    stop();
  }

  // 正转
  // duty: 0~255 占空比 (数值越大速度越快)
  void forward(int duty = 127) {
    duty = constrain(duty, 0, 255);
    digitalWrite(_phPin, HIGH);
    analogWrite(_enPin, duty);
  }

  // 反转
  // duty: 0~255 占空比
  void backward(int duty = 127) {
    duty = constrain(duty, 0, 255);
    digitalWrite(_phPin, LOW);
    analogWrite(_enPin, duty);
  }

  // 停止
  void stop() {
    analogWrite(_enPin, 0);
  }
};

#endif