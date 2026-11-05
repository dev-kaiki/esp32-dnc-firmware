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

  // Deixa um nome digitado pelo operador utilizavel como nome de rede.
  //
  // Nome de rede nao aceita espaco, acento nem ponto (RFC 1123). Um nome
  // invalido nao da erro: o mDNS simplesmente nao anuncia e o DHCP ignora --
  // falha silenciosa, que e a pior de diagnosticar. Entao o saneamento e feito
  // aqui, e o operador ve na tela o nome que de fato ficou.
  static String sanitizarNomeDeEquipamento(const String& bruto);

  AccessConfig loadAccess();
  void saveAccess(const AccessConfig& cfg);

  // True enquanto QUALQUER uma das tres senhas ainda for a de fabrica.
  //
  // Existe porque "cada equipamento com a sua senha" so vale se alguem trocar.
  // Sem um aviso, o aparelho sai da bancada seguro no papel e igual a todos os
  // outros na pratica -- e ninguem descobre ate acontecer.
  bool usandoSenhasDeFabrica(const AccessConfig& cfg);

private:
  Preferences _prefs;
  IPAddress readIP(const char* key, IPAddress def);
  void writeIP(const char* key, IPAddress ip);
};
