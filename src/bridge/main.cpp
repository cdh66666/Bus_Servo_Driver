#include <Arduino.h>
#include <HardwareSerial.h>
#include "HLS3606Emu.h"
#include "BoardServoHardware.h"

// Bridge firmware:
// - USB CDC is a transparent Feetech official-tool bridge.
// - This same board also appears on the bus/tool as a virtual HLS3606-like servo.
// - Default virtual ID is 2; the real factory HLS3915M currently uses ID 1.

static constexpr uint32_t USB_BAUD = 115200;
static constexpr uint32_t DEFAULT_BUS_BAUD = 1000000;
static constexpr uint32_t SEARCH_TIMEOUT_MS = 8;
static constexpr uint32_t KNOWN_TIMEOUT_MS = 12;
static constexpr uint32_t SEARCH_BAUDS[] = {1000000, 115200};
static constexpr uint8_t KNOWN_FACTORY_HLS_ID = 1;
static constexpr uint16_t HLS_DEFAULT_GOAL_SPEED = 80;
static constexpr uint16_t HLS_DEFAULT_GOAL_PWM = 420;

HardwareSerial BusSerial(1);
HLS3606Emu localServo(2, 0, "hls2m");
BoardServoHardware localHardware(localServo, 0, "hls2hw", 1, 1.0f, 0.25f, 0.64f);

typedef void (*PacketHandler)(uint8_t, uint8_t, const uint8_t *, const uint8_t *, uint8_t);

struct PacketParser {
  uint8_t state = 0;
  uint8_t id = 0;
  uint8_t len = 0;
  uint8_t index = 0;
  uint8_t data[HLS_DATA_MAX] = {};
  uint8_t raw[HLS_DATA_MAX + 6] = {};
  uint8_t rawLen = 0;
};

PacketParser usbParser;
PacketParser busParser;
uint8_t lastForwarded[HLS_DATA_MAX + 6] = {};
uint8_t lastForwardedLen = 0;
uint32_t busBaud = 0;
uint32_t idBaud[254] = {};
bool captureActive = false;
bool captureReady = false;
uint8_t captureExpectedId = 0;
uint8_t capturedRaw[HLS_DATA_MAX + 6] = {};
uint8_t capturedRawLen = 0;

void writeUsbPacket(const uint8_t *data, size_t len) {
  Serial.write(data, len);
  Serial.flush();
}

void writeBusPacket(const uint8_t *data, size_t len) {
  BusSerial.write(data, len);
  BusSerial.flush();
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

void drainBusInput() {
  const uint32_t start = micros();
  while (micros() - start < 600) {
    while (BusSerial.available() > 0) {
      BusSerial.read();
    }
    delayMicroseconds(50);
  }
}

bool sameAsLastForwarded(const uint8_t *raw, uint8_t len) {
  return len == lastForwardedLen && memcmp(raw, lastForwarded, len) == 0;
}

bool addCandidate(uint32_t *candidates, uint8_t &count, uint32_t baud) {
  if (baud == 0) return false;
  for (uint8_t i = 0; i < count; ++i) {
    if (candidates[i] == baud) return false;
  }
  candidates[count++] = baud;
  return true;
}

uint8_t buildCandidates(uint8_t id, uint32_t *candidates, uint8_t maxCandidates) {
  uint8_t count = 0;
  if (id < sizeof(idBaud) / sizeof(idBaud[0])) {
    addCandidate(candidates, count, idBaud[id]);
  }
  addCandidate(candidates, count, busBaud);
  for (uint8_t i = 0; i < sizeof(SEARCH_BAUDS) / sizeof(SEARCH_BAUDS[0]) && count < maxCandidates; ++i) {
    addCandidate(candidates, count, SEARCH_BAUDS[i]);
  }
  return count;
}

void rememberResponseBaud(uint8_t id, uint32_t baud) {
  if (id < sizeof(idBaud) / sizeof(idBaud[0])) {
    idBaud[id] = baud;
  }
}

void rememberWriteSideEffects(uint8_t oldId, uint8_t inst, const uint8_t *params,
                              uint8_t paramLen, uint32_t baud) {
  if (oldId >= sizeof(idBaud) / sizeof(idBaud[0])) return;
  if ((inst != HLS_INST_WRITE && inst != HLS_INST_REG_WRITE) || paramLen < 2) return;

  const uint8_t addr = params[0];
  const uint8_t dataLen = paramLen - 1;
  uint8_t affectedId = oldId;
  uint32_t affectedBaud = baud;
  if (addr <= HLS_REG_ID && HLS_REG_ID < addr + dataLen) {
    const uint8_t newId = params[1 + (HLS_REG_ID - addr)];
    if (newId < sizeof(idBaud) / sizeof(idBaud[0])) {
      affectedId = newId;
    }
  }
  if (addr <= HLS_REG_BAUD && HLS_REG_BAUD < addr + dataLen) {
    const uint8_t newBaudCode = params[1 + (HLS_REG_BAUD - addr)];
    affectedBaud = HLS3606Emu::baudFromCode(newBaudCode);
  }
  idBaud[affectedId] = affectedBaud;
}

bool writeTouches(uint8_t addr, uint8_t dataLen, uint8_t reg, uint8_t width = 1) {
  if (dataLen == 0) return false;
  const uint16_t writeStart = addr;
  const uint16_t writeEnd = static_cast<uint16_t>(addr) + dataLen;
  const uint16_t regStart = reg;
  const uint16_t regEnd = static_cast<uint16_t>(reg) + width;
  return writeStart < regEnd && regStart < writeEnd;
}

bool getWriteU16(const uint8_t *params, uint8_t paramLen, uint8_t reg, uint16_t &value) {
  if (paramLen < 3) return false;
  const uint8_t addr = params[0];
  const uint8_t dataLen = paramLen - 1;
  if (addr > reg || static_cast<uint16_t>(reg) + 2 > static_cast<uint16_t>(addr) + dataLen) {
    return false;
  }
  const uint8_t offset = 1 + (reg - addr);
  if (offset + 1 >= paramLen) return false;
  value = static_cast<uint16_t>(params[offset]) |
          (static_cast<uint16_t>(params[offset + 1]) << 8);
  return true;
}

void putWriteU16(uint8_t *params, uint8_t paramLen, uint8_t reg, uint16_t value) {
  if (paramLen < 3) return;
  const uint8_t addr = params[0];
  const uint8_t dataLen = paramLen - 1;
  if (addr > reg || static_cast<uint16_t>(reg) + 2 > static_cast<uint16_t>(addr) + dataLen) {
    return;
  }
  const uint8_t offset = 1 + (reg - addr);
  if (offset + 1 >= paramLen) return;
  params[offset] = static_cast<uint8_t>(value & 0xFF);
  params[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void buildRawInstruction(uint8_t id, uint8_t inst, const uint8_t *params, uint8_t paramLen,
                         uint8_t *raw, uint8_t &rawLen) {
  const uint8_t packetLen = paramLen + 2;
  raw[0] = 0xFF;
  raw[1] = 0xFF;
  raw[2] = id;
  raw[3] = packetLen;
  raw[4] = inst;
  memcpy(raw + 5, params, paramLen);
  raw[5 + paramLen] = HLS3606Emu::checksum(raw + 2, static_cast<size_t>(packetLen + 1));
  rawLen = static_cast<uint8_t>(6 + paramLen);
}

bool normalizePhysicalHlsMotionWrite(uint8_t id, uint8_t inst, const uint8_t *params,
                                     uint8_t paramLen, uint8_t *patchedParams,
                                     uint8_t &patchedParamLen,
                                     uint8_t *patchedRaw, uint8_t &patchedRawLen) {
  if (id != KNOWN_FACTORY_HLS_ID) return false;
  if (inst != HLS_INST_WRITE && inst != HLS_INST_REG_WRITE) return false;
  if (paramLen < 3) return false;

  const uint8_t addr = params[0];
  const uint8_t dataLen = paramLen - 1;
  if (!writeTouches(addr, dataLen, HLS_REG_GOAL_POSITION_L, 2)) return false;

  uint16_t speed = 0;
  const bool hasSpeed = getWriteU16(params, paramLen, HLS_REG_RUN_SPEED_L, speed);
  if (hasSpeed && speed != 0) return false;

  uint8_t newParamLen = paramLen;
  if (!hasSpeed) {
    const uint8_t requiredDataLen =
        static_cast<uint8_t>((HLS_REG_RUN_SPEED_L + 2) - addr);
    newParamLen = static_cast<uint8_t>(1 + requiredDataLen);
  }
  if (newParamLen > HLS_DATA_MAX) return false;

  memset(patchedParams, 0, HLS_DATA_MAX);
  patchedParams[0] = addr;
  for (uint8_t i = 1; i < paramLen; ++i) patchedParams[i] = params[i];
  patchedParamLen = newParamLen;
  putWriteU16(patchedParams, patchedParamLen, HLS_REG_GOAL_CURRENT_L, HLS_DEFAULT_GOAL_PWM);
  putWriteU16(patchedParams, patchedParamLen, HLS_REG_RUN_SPEED_L, HLS_DEFAULT_GOAL_SPEED);
  buildRawInstruction(id, inst, patchedParams, patchedParamLen, patchedRaw, patchedRawLen);
  return true;
}

bool waitForCapturedResponse(uint8_t expectedId, uint32_t timeoutMs);
bool sendToBusAtBaud(uint8_t id, uint8_t inst, const uint8_t *params, uint8_t paramLen,
                     const uint8_t *raw, uint8_t rawLen, uint32_t baud, uint32_t timeoutMs) {
  beginBus(baud);
  drainBusInput();
  const bool knownAtThisBaud = id < sizeof(idBaud) / sizeof(idBaud[0]) && idBaud[id] == baud;
  memcpy(lastForwarded, raw, rawLen);
  lastForwardedLen = rawLen;
  capturedRawLen = 0;
  captureReady = false;
  captureExpectedId = id;
  captureActive = true;

  writeBusPacket(raw, rawLen);
  const bool gotResponse = waitForCapturedResponse(id, timeoutMs);
  captureActive = false;

  if (gotResponse) {
    writeUsbPacket(capturedRaw, capturedRawLen);
    rememberResponseBaud(capturedRaw[2], baud);
    rememberWriteSideEffects(id, inst, params, paramLen, baud);
    return true;
  }

  if (knownAtThisBaud) {
    rememberWriteSideEffects(id, inst, params, paramLen, baud);
  }
  return false;
}

void sendBroadcastToKnownBauds(const uint8_t *raw, uint8_t rawLen) {
  uint32_t candidates[8] = {};
  uint8_t count = 0;
  addCandidate(candidates, count, busBaud);
  for (uint8_t i = 0; i < sizeof(SEARCH_BAUDS) / sizeof(SEARCH_BAUDS[0]); ++i) {
    addCandidate(candidates, count, SEARCH_BAUDS[i]);
  }
  for (uint8_t sid = 0; sid < sizeof(idBaud) / sizeof(idBaud[0]); ++sid) {
    if (count >= sizeof(candidates) / sizeof(candidates[0])) break;
    addCandidate(candidates, count, idBaud[sid]);
  }
  for (uint8_t i = 0; i < count; ++i) {
    beginBus(candidates[i]);
    drainBusInput();
    memcpy(lastForwarded, raw, rawLen);
    lastForwardedLen = rawLen;
    writeBusPacket(raw, rawLen);
    delay(2);
  }
}

void proxyPacketToBus(uint8_t id, uint8_t inst, const uint8_t *params, uint8_t paramLen,
                      const uint8_t *raw, uint8_t rawLen) {
  if (id == HLS_BROADCAST_ID) {
    sendBroadcastToKnownBauds(raw, rawLen);
    return;
  }

  uint8_t patchedParams[HLS_DATA_MAX] = {};
  uint8_t patchedParamLen = 0;
  uint8_t patchedRaw[HLS_DATA_MAX + 6] = {};
  uint8_t patchedRawLen = 0;
  if (normalizePhysicalHlsMotionWrite(id, inst, params, paramLen,
                                      patchedParams, patchedParamLen,
                                      patchedRaw, patchedRawLen)) {
    params = patchedParams;
    paramLen = patchedParamLen;
    raw = patchedRaw;
    rawLen = patchedRawLen;
  }

  uint32_t candidates[6] = {};
  const uint8_t count = buildCandidates(id, candidates, sizeof(candidates) / sizeof(candidates[0]));
  for (uint8_t i = 0; i < count; ++i) {
    const uint32_t timeoutMs = (id < sizeof(idBaud) / sizeof(idBaud[0]) && idBaud[id] == candidates[i])
                                   ? KNOWN_TIMEOUT_MS
                                   : SEARCH_TIMEOUT_MS;
    if (sendToBusAtBaud(id, inst, params, paramLen, raw, rawLen, candidates[i], timeoutMs)) {
      return;
    }
  }
}

void handleUsbPacket(uint8_t id, uint8_t len, const uint8_t *data, const uint8_t *raw, uint8_t rawLen) {
  if (len < 2) return;
  const uint8_t inst = data[0];
  const uint8_t *params = data + 1;
  const uint8_t paramLen = len - 2;

  if (localServo.accepts(id)) {
    localServo.processInstruction(id, inst, params, paramLen, writeUsbPacket);
  }

  // Broadcast packets must continue to the physical bus.  Local-only packets do not need to.
  if (id != localServo.id()) {
    proxyPacketToBus(id, inst, params, paramLen, raw, rawLen);
  }
}

void handleBusPacket(uint8_t id, uint8_t len, const uint8_t *data, const uint8_t *raw, uint8_t rawLen) {
  (void)len;
  (void)data;
  if (sameAsLastForwarded(raw, rawLen)) return;
  if (captureActive && (captureExpectedId == HLS_BROADCAST_ID || id == captureExpectedId)) {
    memcpy(capturedRaw, raw, rawLen);
    capturedRawLen = rawLen;
    captureReady = true;
    return;
  }
  // Forward real servo status packets back to the official PC tool.
  writeUsbPacket(raw, rawLen);
}

void feedParser(PacketParser &p, uint8_t b, PacketHandler handler) {
  switch (p.state) {
    case 0:
      if (b == 0xFF) {
        p.state = 1;
        p.rawLen = 0;
        p.raw[p.rawLen++] = b;
      }
      break;
    case 1:
      if (b == 0xFF) {
        p.state = 2;
        p.raw[p.rawLen++] = b;
      } else {
        p.state = 0;
      }
      break;
    case 2:
      p.id = b;
      p.raw[p.rawLen++] = b;
      p.state = 3;
      break;
    case 3:
      p.len = b;
      p.index = 0;
      p.raw[p.rawLen++] = b;
      if (p.len < 2 || p.len > HLS_DATA_MAX) p.state = 0;
      else p.state = 4;
      break;
    case 4:
      p.data[p.index++] = b;
      if (p.rawLen < sizeof(p.raw)) p.raw[p.rawLen++] = b;
      if (p.index >= p.len) {
        uint8_t sumBuf[HLS_DATA_MAX + 2];
        sumBuf[0] = p.id;
        sumBuf[1] = p.len;
        memcpy(sumBuf + 2, p.data, p.len - 1);
        const uint8_t expected = HLS3606Emu::checksum(sumBuf, p.len + 1);
        if (expected == p.data[p.len - 1]) {
          handler(p.id, p.len, p.data, p.raw, p.rawLen);
        }
        p.state = 0;
      }
      break;
  }
}

bool waitForCapturedResponse(uint8_t expectedId, uint32_t timeoutMs) {
  (void)expectedId;
  const uint32_t timeoutUs = timeoutMs * 1000UL;
  const uint32_t start = micros();
  while (micros() - start < timeoutUs) {
    while (BusSerial.available() > 0) {
      feedParser(busParser, static_cast<uint8_t>(BusSerial.read()), handleBusPacket);
      if (captureReady) return true;
    }
    delayMicroseconds(50);
  }
  return captureReady;
}

void updateStatusLed() {
  static uint32_t lastToggleMs = 0;
  static bool ledOn = false;
  const uint32_t now = millis();
  const bool hardwareOk = localHardware.encoderOnline() && localHardware.errorFlags() == 0;
  const uint32_t intervalMs = hardwareOk ? 500 : 120;
  if (now - lastToggleMs < intervalMs) return;

  lastToggleMs = now;
  ledOn = !ledOn;
  digitalWrite(PIN_LED, ledOn ? HIGH : LOW);
}

void setup() {
  Serial.begin(USB_BAUD);
  localServo.begin();
  localHardware.begin();
  idBaud[1] = 1000000;          // The powered HLS3915M has been verified as ID 1 at 1 Mbps.
  idBaud[3] = DEFAULT_BUS_BAUD; // COM58 virtual servo default.
  beginBus(DEFAULT_BUS_BAUD);
}

void loop() {
  localHardware.update();
  updateStatusLed();
  while (Serial.available() > 0) {
    feedParser(usbParser, static_cast<uint8_t>(Serial.read()), handleUsbPacket);
  }
  while (BusSerial.available() > 0) {
    feedParser(busParser, static_cast<uint8_t>(BusSerial.read()), handleBusPacket);
  }
}
