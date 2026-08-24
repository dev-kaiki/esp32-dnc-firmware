#pragma once
#include <Arduino.h>
#include "Types.h"

class SerialManager {
public:
  bool begin(const DncSerialConfig& cfg);
  bool applyConfig(const DncSerialConfig& cfg);
  void connectLines();
  void disconnectLines();
  bool isConnected() const;
  int readByte();
  bool writeByte(uint8_t b);
  void flush();
  bool ctsOk();
  bool dsrOk();
  bool canTransmit(const DncSerialConfig& cfg, bool xonAllowed);
  uint32_t serialConfigConstant(const DncSerialConfig& cfg);

private:
  bool _connected = false;
  DncSerialConfig _cfg;
};
