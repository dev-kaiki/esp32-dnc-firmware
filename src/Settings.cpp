#include "Settings.h"
#include "../Config.h"

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
  // Nome proprio deste equipamento. Padrao = nome de fabrica, para que uma
  // caixa ja instalada continue funcionando depois da atualizacao.
  cfg.nomeDoEquipamento = sanitizarNomeDeEquipamento(_prefs.getString("hostname", HOSTNAME));
  cfg.ssid = _prefs.getString("ssid", "");
  cfg.password = _prefs.getString("wpass", "");
  // Padrao vazio de proposito: equipamento novo sai sem OTA ligado.
  cfg.otaPassword = _prefs.getString("otapass", "");
  cfg.dhcp = _prefs.getBool("dhcp", true);
  cfg.ip = readIP("ip", IPAddress(192, 168, 1, 220));
  cfg.gateway = readIP("gw", IPAddress(192, 168, 1, 1));
  cfg.subnet = readIP("mask", IPAddress(255, 255, 255, 0));
  cfg.dns = readIP("dns", IPAddress(8, 8, 8, 8));
  return cfg;
}

String Settings::sanitizarNomeDeEquipamento(const String& bruto) {
  String n;
  n.reserve(bruto.length());

  for (size_t i = 0; i < bruto.length(); ++i) {
    char c = bruto[i];
    if (c >= 'A' && c <= 'Z') c = char(c - 'A' + 'a');   // minusculas

    const bool valido  = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
    const bool ultimoEhTraco = (n.length() > 0 && n[n.length() - 1] == '-');

    if (valido) {
      n += c;
    } else if (!ultimoEhTraco) {
      // Tudo o que nao serve -- espaco, acento, ponto, sinal -- vira UM traco.
      //
      // E o traco digitado direto passa pela MESMA regra, que era o furo aqui:
      // antes so os caracteres convertidos respeitavam o "nunca dois seguidos",
      // entao "centro---de" atravessava inteiro. Fora de feio, "--" na terceira
      // e quarta posicao e reservado para nomes internacionalizados.
      n += '-';
    }
  }

  while (n.length() > 0 && n[n.length() - 1] == '-') n.remove(n.length() - 1);
  while (n.length() > 0 && n[0] == '-') n.remove(0, 1);

  // Teto de 31: e o limite pratico de nome de host em rede local.
  if (n.length() > 31) {
    n = n.substring(0, 31);
    while (n.length() > 0 && n[n.length() - 1] == '-') n.remove(n.length() - 1);
  }

  // Sobrou nada de utilizavel: volta para o nome de fabrica, que ao menos
  // funciona. Um equipamento sem nome nao entra na rede.
  if (n.length() == 0) return String(HOSTNAME);
  return n;
}

AccessConfig Settings::loadAccess() {
  AccessConfig cfg;
  // O padrao e a senha antiga de codigo. Assim um equipamento ja instalado
  // continua entrando depois da atualizacao, em vez de ficar inacessivel -- e a
  // troca vira uma acao do vendedor, nao uma emergencia do cliente.
  cfg.adminPass = _prefs.getString("padm",  DEFAULT_ADMIN_PASS);
  cfg.operPass  = _prefs.getString("pope",  DEFAULT_OPER_PASS);
  cfg.viewPass  = _prefs.getString("pview", DEFAULT_VIEW_PASS);
  return cfg;
}

void Settings::saveAccess(const AccessConfig& cfg) {
  _prefs.putString("padm",  cfg.adminPass);
  _prefs.putString("pope",  cfg.operPass);
  _prefs.putString("pview", cfg.viewPass);
}

bool Settings::usandoSenhasDeFabrica(const AccessConfig& cfg) {
  return cfg.adminPass == DEFAULT_ADMIN_PASS
      || cfg.operPass  == DEFAULT_OPER_PASS
      || cfg.viewPass  == DEFAULT_VIEW_PASS;
}

void Settings::saveNetwork(const NetworkConfig& cfg) {
  _prefs.putString("hostname", cfg.nomeDoEquipamento);
  _prefs.putString("ssid", cfg.ssid);
  _prefs.putString("wpass", cfg.password);
  _prefs.putString("otapass", cfg.otaPassword);
  _prefs.putBool("dhcp", cfg.dhcp);
  writeIP("ip", cfg.ip);
  writeIP("gw", cfg.gateway);
  writeIP("mask", cfg.subnet);
  writeIP("dns", cfg.dns);
}
