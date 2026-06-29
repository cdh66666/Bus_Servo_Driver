#include <Arduino.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include "HLS3606Emu.h"
#include "BoardServoHardware.h"

// Node firmware:
// Pure virtual HLS3606-like servo on the shared Feetech bus.
// Default ID is 3; the real factory HLS3915M currently uses ID 1.

static constexpr uint32_t USB_BAUD = 115200;
static constexpr uint32_t DEFAULT_BUS_BAUD = 1000000;

HardwareSerial BusSerial(1);
HLS3606Emu servo(3, 0, "hls3m");
BoardServoHardware servoHardware(servo, 0, "hls3hw", -1, 0.30f, 0.14f);
uint32_t busBaud = 0;

struct PacketParser {
  uint8_t state = 0;
  uint8_t id = 0;
  uint8_t len = 0;
  uint8_t index = 0;
  uint8_t data[HLS_DATA_MAX] = {};
};

PacketParser parser;
String usbLine;

int32_t parseNumber(const String &text, int32_t fallback = 0) {
  char *end = nullptr;
  const long value = strtol(text.c_str(), &end, 0);
  return end == text.c_str() ? fallback : value;
}

int16_t decodeSignedMagnitude15(uint16_t value) {
  const int16_t magnitude = static_cast<int16_t>(value & 0x7FFF);
  return (value & 0x8000) != 0 ? static_cast<int16_t>(-magnitude) : magnitude;
}

int16_t decodeLoad(uint16_t value) {
  const int16_t magnitude = static_cast<int16_t>(value & 0x03FF);
  return (value & 0x0400) != 0 ? static_cast<int16_t>(-magnitude) : magnitude;
}

const char *encoderName(uint8_t type) {
  switch (type) {
    case static_cast<uint8_t>(EncoderKind::MT6701):
      return "MT6701";
    case static_cast<uint8_t>(EncoderKind::AS5600):
      return "AS5600";
    default:
      return "NONE";
  }
}

void beginBus(uint32_t baud) {
  if (busBaud == baud) return;
  if (busBaud == 0) {
    busBaud = baud;
    BusSerial.begin(busBaud, SERIAL_8N1, PIN_BUS_RX, PIN_BUS_TX);
  } else {
    BusSerial.flush();
    BusSerial.updateBaudRate(baud);
    busBaud = baud;
  }
  delayMicroseconds(300);
}

void writeBusPacket(const uint8_t *data, size_t len) {
  BusSerial.write(data, len);
  BusSerial.flush();
}

void syncBusBaudFromServo() {
  const uint32_t desiredBaud = HLS3606Emu::baudFromCode(servo.baudCode());
  if (desiredBaud != busBaud) {
    BusSerial.flush();
    delay(5);
    beginBus(desiredBaud);
  }
}

void feedBusByte(uint8_t b) {
  switch (parser.state) {
    case 0:
      parser.state = (b == 0xFF) ? 1 : 0;
      break;
    case 1:
      parser.state = (b == 0xFF) ? 2 : 0;
      break;
    case 2:
      parser.id = b;
      parser.state = 3;
      break;
    case 3:
      parser.len = b;
      parser.index = 0;
      if (parser.len < 2 || parser.len > HLS_DATA_MAX) parser.state = 0;
      else parser.state = 4;
      break;
    case 4:
      parser.data[parser.index++] = b;
      if (parser.index >= parser.len) {
        uint8_t sumBuf[HLS_DATA_MAX + 2];
        sumBuf[0] = parser.id;
        sumBuf[1] = parser.len;
        memcpy(sumBuf + 2, parser.data, parser.len - 1);
        const uint8_t expected = HLS3606Emu::checksum(sumBuf, parser.len + 1);
        if (expected == parser.data[parser.len - 1]) {
          const uint8_t inst = parser.data[0];
          const uint8_t *params = parser.data + 1;
          const uint8_t paramLen = parser.len - 2;
          servo.processInstruction(parser.id, inst, params, paramLen, writeBusPacket);
          syncBusBaudFromServo();
        }
        parser.state = 0;
      }
      break;
  }
}

void printStatus() {
  Serial.println();
  Serial.println("=== virtual HLS servo ===");
  Serial.printf("id=%u baud=%lu ok=%lu status=%lu\n",
                servo.id(), HLS3606Emu::baudFromCode(servo.baudCode()),
                servo.goodPackets(), servo.statusPackets());
  Serial.println("=========================");
}

void printHelp() {
  Serial.println("commands: status, diag, phase 0|1, torque 0|1, read <addr> <len>");
}

void printDiag() {
  const int16_t position = decodeSignedMagnitude15(servo.regU16(HLS_REG_PRESENT_POSITION_L));
  const int16_t speed = decodeSignedMagnitude15(servo.regU16(HLS_REG_PRESENT_SPEED_L));
  const int16_t load = decodeLoad(servo.regU16(HLS_REG_PRESENT_LOAD_L));
  const int16_t current = decodeSignedMagnitude15(servo.regU16(HLS_REG_PRESENT_CURRENT_L));
  const int16_t effortMilli = static_cast<int16_t>(servo.regU16(HLS_DIAG_LAST_EFFORT_MILLI_L));
  Serial.println();
  Serial.println("=== board diag ===");
  Serial.printf("id=%u baud=%lu mode=%u torque=%u setting=0x%02X error=0x%02X moving=%u\n",
                servo.id(), HLS3606Emu::baudFromCode(servo.baudCode()), servo.mode(),
                servo.reg(HLS_REG_TORQUE_ENABLE), servo.reg(HLS_REG_SETTING),
                servo.reg(HLS_REG_ERROR), servo.reg(HLS_REG_MOVING));
  Serial.printf("pos=%d speed=%d load=%d voltage=%.1fV temp=%u current=%d mA\n",
                position, speed, load, servo.reg(HLS_REG_PRESENT_VOLTAGE) / 10.0f,
                servo.reg(HLS_REG_PRESENT_TEMP), current);
  Serial.printf("current_ipropi=%u mV current_raw=%u mA effort=%d/1000\n",
                servo.regU16(HLS_DIAG_CURRENT_RAW_MV_L),
                servo.regU16(HLS_DIAG_CURRENT_MA_L), effortMilli);
  Serial.printf("encoder=%s raw14=%u first_i2c=0x%02X active_i2c=0x%02X enc_status=0x%02X\n",
                encoderName(servo.reg(HLS_DIAG_ENCODER_TYPE)), servo.regU16(HLS_DIAG_RAW14_L),
                servo.reg(HLS_DIAG_FIRST_I2C), servo.reg(HLS_DIAG_ACTIVE_I2C),
                servo.reg(HLS_DIAG_ENCODER_STATUS));
  Serial.printf("motor_fault=%u hw_error=0x%02X\n",
                servoHardware.motorFaulted(), servoHardware.errorFlags());
  Serial.println("==================");
}

void printRegisterRange(uint8_t addr, uint8_t len) {
  Serial.printf("reg 0x%02X:", addr);
  for (uint8_t i = 0; i < len; ++i) {
    Serial.printf(" %02X", servo.reg(addr + i));
  }
  Serial.println();
}

void handleUsbLine(String line) {
  line.trim();
  line.toLowerCase();
  if (line == "status") {
    printStatus();
  } else if (line == "help" || line == "?") {
    printHelp();
  } else if (line == "diag") {
    printDiag();
  } else if (line.startsWith("phase ")) {
    const uint8_t phase = parseNumber(line.substring(6), 0) != 0 ? 1 : 0;
    uint8_t setting = servo.reg(HLS_REG_SETTING);
    setting = phase ? (setting | 0x01) : (setting & static_cast<uint8_t>(~0x01));
    servo.setReg(HLS_REG_SETTING, setting);
    servo.savePersistentSettings();
    Serial.printf("phase invert=%u saved, setting=0x%02X\n", phase, setting);
  } else if (line.startsWith("torque ")) {
    const uint8_t enabled = parseNumber(line.substring(7), 0) != 0 ? 1 : 0;
    servo.setReg(HLS_REG_TORQUE_ENABLE, enabled);
    Serial.printf("torque=%u\n", enabled);
  } else if (line.startsWith("read ")) {
    const int firstSpace = line.indexOf(' ', 5);
    if (firstSpace < 0) {
      Serial.println("usage: read <addr> <len>");
      return;
    }
    const uint8_t addr = static_cast<uint8_t>(parseNumber(line.substring(5, firstSpace), 0));
    const uint8_t len = static_cast<uint8_t>(constrain(parseNumber(line.substring(firstSpace + 1), 1), 1, 32));
    printRegisterRange(addr, len);
  } else if (line.length() > 0) {
    printHelp();
  }
}

void pollUsb() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      handleUsbLine(usbLine);
      usbLine = "";
    } else if (usbLine.length() < 80) {
      usbLine += c;
    }
  }
}

void updateStatusLed() {
  static uint32_t lastToggleMs = 0;
  static bool ledOn = false;
  const uint32_t now = millis();
  const bool hardwareOk = servoHardware.encoderOnline() && servoHardware.errorFlags() == 0;
  const uint32_t intervalMs = hardwareOk ? 500 : 120;
  if (now - lastToggleMs < intervalMs) return;

  lastToggleMs = now;
  ledOn = !ledOn;
  digitalWrite(PIN_LED, ledOn ? HIGH : LOW);
}

void setup() {
  Serial.begin(USB_BAUD);
  servo.begin();
  servoHardware.begin();
  beginBus(HLS3606Emu::baudFromCode(servo.baudCode()));
  printStatus();
}

void loop() {
  servoHardware.update();
  updateStatusLed();
  while (BusSerial.available() > 0) {
    feedBusByte(static_cast<uint8_t>(BusSerial.read()));
  }
  pollUsb();
}
