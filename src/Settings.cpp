#include "Settings.h"

bool Settings::begin() {
  return _prefs.begin("smi-dnc", false);
}

DncSerialConfig Settings::loadSerial() {
  DncSerialConfig cfg;
  cfg.baudRate = _prefs.getUInt("baud", cfg.baudRate);
  cfg.dataBits = _prefs.getUChar("data", cfg.dataBits);
  cfg.parity = (ParityMode)_prefs.getUChar("parity", (uint8_t)cfg.parity);
  cfg.stopBits = _prefs.getUChar("stop", cfg.stopBits);
  cfg.timeoutMs = _prefs.getUInt("timeout", cfg.timeoutMs);
  cfg.flow = (FlowControlMode)_prefs.getUChar("flow", (uint8_t)cfg.flow);

  if (cfg.dataBits < 5 || cfg.dataBits > 8) cfg.dataBits = 7;
  if (cfg.stopBits != 1 && cfg.stopBits != 2) cfg.stopBits = 1;
  if (cfg.baudRate < 300 || cfg.baudRate > 230400) cfg.baudRate = 9600;
  if (cfg.timeoutMs < 100) cfg.timeoutMs = 1000;
  if (cfg.parity > PARITY_ODD) cfg.parity = PARITY_NONE;
  if (cfg.flow > FLOW_DTRDSR) cfg.flow = FLOW_NONE;
  return cfg;
}

void Settings::saveSerial(const DncSerialConfig& cfg) {
  _prefs.putUInt("baud", cfg.baudRate);
  _prefs.putUChar("data", cfg.dataBits);
  _prefs.putUChar("parity", (uint8_t)cfg.parity);
  _prefs.putUChar("stop", cfg.stopBits);
  _prefs.putUInt("timeout", cfg.timeoutMs);
  _prefs.putUChar("flow", (uint8_t)cfg.flow);
}

IPAddress Settings::readIP(const char* key, IPAddress def) {
  String s = _prefs.getString(key, "");
  IPAddress ip;
  if (s.length() && ip.fromString(s)) return ip;
  return def;
}

void Settings::writeIP(const char* key, IPAddress ip) {
  _prefs.putString(key, ip.toString());
}

NetworkConfig Settings::loadNetwork() {
  NetworkConfig cfg;
  cfg.ssid = _prefs.getString("ssid", "");
  cfg.password = _prefs.getString("wpass", "");
  cfg.dhcp = _prefs.getBool("dhcp", true);
  cfg.ip = readIP("ip", IPAddress(192, 168, 1, 220));
  cfg.gateway = readIP("gw", IPAddress(192, 168, 1, 1));
  cfg.subnet = readIP("mask", IPAddress(255, 255, 255, 0));
  cfg.dns = readIP("dns", IPAddress(8, 8, 8, 8));
  return cfg;
}

void Settings::saveNetwork(const NetworkConfig& cfg) {
  _prefs.putString("ssid", cfg.ssid);
  _prefs.putString("wpass", cfg.password);
  _prefs.putBool("dhcp", cfg.dhcp);
  writeIP("ip", cfg.ip);
  writeIP("gw", cfg.gateway);
  writeIP("mask", cfg.subnet);
  writeIP("dns", cfg.dns);
}
