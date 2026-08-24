#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "Types.h"

class Settings {
public:
  bool begin();
  DncSerialConfig loadSerial();
  void saveSerial(const DncSerialConfig& cfg);
  NetworkConfig loadNetwork();
  void saveNetwork(const NetworkConfig& cfg);

private:
  Preferences _prefs;
  IPAddress readIP(const char* key, IPAddress def);
  void writeIP(const char* key, IPAddress ip);
};
