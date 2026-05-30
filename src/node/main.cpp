#include <Arduino.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include "HLS3606Emu.h"
#include "BoardServoHardware.h"

// COM58 firmware:
// Pure virtual HLS3606-like servo on the shared Feetech bus.
// Default ID is 3; the real factory HLS3915M currently uses ID 1.

static constexpr int8_t PIN_BUS_RX = 20;
static constexpr int8_t PIN_BUS_TX = 21;
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

void handleUsbLine(String line) {
  line.trim();
  line.toLowerCase();
  if (line == "status") {
    printStatus();
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

void setup() {
  Serial.begin(USB_BAUD);
  servo.begin();
  servoHardware.begin();
  beginBus(HLS3606Emu::baudFromCode(servo.baudCode()));
  printStatus();
}

void loop() {
  servoHardware.update();
  while (BusSerial.available() > 0) {
    feedBusByte(static_cast<uint8_t>(BusSerial.read()));
  }
  pollUsb();
}
