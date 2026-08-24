#include "SerialManager.h"
#include "../Config.h"

HardwareSerial DncSerial(2);

uint32_t SerialManager::serialConfigConstant(const DncSerialConfig& cfg) {
  // Mapeia configuracao flexivel para constantes aceitas pelo core Arduino-ESP32.
  // Nem todo CNC usa 5/6 bits; Fanuc classico costuma usar 7E1 ou 7E2.
  if (cfg.dataBits == 5) {
    if (cfg.parity == PARITY_EVEN && cfg.stopBits == 2) return SERIAL_5E2;
    if (cfg.parity == PARITY_EVEN) return SERIAL_5E1;
    if (cfg.parity == PARITY_ODD  && cfg.stopBits == 2) return SERIAL_5O2;
    if (cfg.parity == PARITY_ODD) return SERIAL_5O1;
    return cfg.stopBits == 2 ? SERIAL_5N2 : SERIAL_5N1;
  }
  if (cfg.dataBits == 6) {
    if (cfg.parity == PARITY_EVEN && cfg.stopBits == 2) return SERIAL_6E2;
    if (cfg.parity == PARITY_EVEN) return SERIAL_6E1;
    if (cfg.parity == PARITY_ODD  && cfg.stopBits == 2) return SERIAL_6O2;
    if (cfg.parity == PARITY_ODD) return SERIAL_6O1;
    return cfg.stopBits == 2 ? SERIAL_6N2 : SERIAL_6N1;
  }
  if (cfg.dataBits == 7) {
    if (cfg.parity == PARITY_EVEN && cfg.stopBits == 2) return SERIAL_7E2;
    if (cfg.parity == PARITY_EVEN) return SERIAL_7E1;
    if (cfg.parity == PARITY_ODD  && cfg.stopBits == 2) return SERIAL_7O2;
    if (cfg.parity == PARITY_ODD) return SERIAL_7O1;
    return cfg.stopBits == 2 ? SERIAL_7N2 : SERIAL_7N1;
  }
  // default 8 bits
  if (cfg.parity == PARITY_EVEN && cfg.stopBits == 2) return SERIAL_8E2;
  if (cfg.parity == PARITY_EVEN) return SERIAL_8E1;
  if (cfg.parity == PARITY_ODD  && cfg.stopBits == 2) return SERIAL_8O2;
  if (cfg.parity == PARITY_ODD) return SERIAL_8O1;
  return cfg.stopBits == 2 ? SERIAL_8N2 : SERIAL_8N1;
}

bool SerialManager::begin(const DncSerialConfig& cfg) {
  pinMode(PIN_RS232_RTS, OUTPUT);
  pinMode(PIN_RS232_DTR, OUTPUT);
  pinMode(PIN_RS232_CTS, INPUT_PULLUP);
  pinMode(PIN_RS232_DSR, INPUT_PULLUP);
  disconnectLines();
  return applyConfig(cfg);
}

bool SerialManager::applyConfig(const DncSerialConfig& cfg) {
  _cfg = cfg;
  DncSerial.end();
  delay(30);
  DncSerial.begin(cfg.baudRate, serialConfigConstant(cfg), PIN_RS232_RX, PIN_RS232_TX, false, cfg.timeoutMs);
  DncSerial.setTimeout(cfg.timeoutMs);
  _connected = true;
  connectLines();
  return true;
}

void SerialManager::connectLines() {
  digitalWrite(PIN_RS232_RTS, FLOW_OUTPUT_ACTIVE_LEVEL);
  digitalWrite(PIN_RS232_DTR, FLOW_OUTPUT_ACTIVE_LEVEL);
  _connected = true;
}

void SerialManager::disconnectLines() {
  digitalWrite(PIN_RS232_RTS, FLOW_OUTPUT_INACTIVE_LEVEL);
  digitalWrite(PIN_RS232_DTR, FLOW_OUTPUT_INACTIVE_LEVEL);
  _connected = false;
}

bool SerialManager::isConnected() const {
  return _connected;
}

int SerialManager::readByte() {
  if (!_connected) return -1;
  if (DncSerial.available() <= 0) return -1;
  return DncSerial.read();
}

bool SerialManager::writeByte(uint8_t b) {
  if (!_connected) return false;
  return DncSerial.write(&b, 1) == 1;
}

void SerialManager::flush() {
  DncSerial.flush();
}

bool SerialManager::ctsOk() {
  return digitalRead(PIN_RS232_CTS) == FLOW_INPUT_ACTIVE_LEVEL;
}

bool SerialManager::dsrOk() {
  return digitalRead(PIN_RS232_DSR) == FLOW_INPUT_ACTIVE_LEVEL;
}

bool SerialManager::canTransmit(const DncSerialConfig& cfg, bool xonAllowed) {
  if (!_connected) return false;
  if (cfg.flow == FLOW_XONXOFF) return xonAllowed;
  if (cfg.flow == FLOW_RTSCTS) return ctsOk();
  if (cfg.flow == FLOW_DTRDSR) return dsrOk();
  return true;
}
