#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
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

  // --- Bloco -----------------------------------------------------------------
  //
  // O motor trabalha em pedacos de dezenas de bytes. Ir de byte em byte fixava
  // o envio em 1000 B/s (uma volta de tarefa por milissegundo) -- menos do que
  // a maquina aceita ja a 19200 baud.
  size_t readBlock(uint8_t* out, size_t maxLen);

  // NUNCA BLOQUEIA: escreve no maximo o que couber no buffer de saida agora, e
  // devolve quantos bytes foram aceitos. Quem chama so pode dar por consumido
  // o que voltou daqui.
  size_t writeBlock(const uint8_t* dados, size_t n);

  // Quanto cabe no buffer de saida sem bloquear.
  // Duas configuracoes valem a mesma coisa no fio? Ver a nota em applyConfig.
  static bool mesmaConfig(const DncSerialConfig& a, const DncSerialConfig& b);

  size_t espacoLivreDeEscrita();

  // Quanto ja foi escrito e ainda nao saiu pelo fio. E o que o agendador usa
  // para nao empurrar em cima de uma fila que a maquina ainda nao consumiu.
  size_t bytesNaSaida();

  // Manda um caractere de controle de fluxo NA FRENTE de tudo.
  //
  // XON e XOFF nao entram na fila de dados de proposito. Enfileirar o XOFF
  // significaria que ele so sairia depois de tudo o que ja esta na fila --
  // exatamente quando ele precisa chegar rapido. E o XON nao pode ficar preso
  // atras da permissao de envio, senao um XOFF anterior impediria de dizer
  // "pode mandar", o que trava a conversa dos dois lados.
  bool enviarControleDeFluxo(uint8_t caractere);

  void flush();
  bool ctsOk();
  bool dsrOk();
  // Verdadeiro quando o freio de XON/XOFF esta a cargo do periferico.
  // O Diagnostico mostra isto: e a diferenca entre parar em um caractere e
  // parar dezenas depois, que e o que a Fanuc conta antes de alarmar 087.
  bool freioXonXoffNoHardware() const { return _freioXonXoffNoHardware; }

  // Arma o freio de fluxo do periferico. Chamado de TODO caminho de
  // applyConfig -- inclusive do atalho de "configuracao igual".
  void configurarFreioDeFluxo(const DncSerialConfig& cfg);

  // Reabre a porta para jogar fora o que ficou preso na saida. Devolve quantos
  // bytes foram descartados. So faz sentido no cancelamento -- ver a nota.
  size_t descartarSaidaPendente();

  bool canTransmit(const DncSerialConfig& cfg, bool xonAllowed);
  uint32_t serialConfigConstant(const DncSerialConfig& cfg);

private:
  // Se a UART ja foi aberta alguma vez. Sem isto, a primeira chamada com a
  // configuracao padrao seria tratada como "igual, nao faz nada" e a porta
  // nunca abriria.
  bool _iniciada = false;

  bool _connected = false;

  // UMA SO TAREFA MEXE NA PORTA POR VEZ.
  //
  // A TaskSerial escreve e le a cada milissegundo; o cancelamento (que vem da
  // TaskWebServer, pelo aplicativo ou pelo portal) REABRE a porta para jogar
  // fora o que ficou preso. Sem trava, as duas mexiam na mesma UART ao mesmo
  // tempo -- end() no meio de um write().
  //
  // Recursivo de proposito: descartarSaidaPendente() chama connectLines() e
  // configurarFreioDeFluxo() ja segurando a trava, e com mutex comum isso
  // travaria o equipamento.
  SemaphoreHandle_t _portaMutex = nullptr;

  // Toma e solta a trava da porta. Seguras antes de begin(), quando o mutex
  // ainda nao existe.
  void travarPorta();
  void destravarPorta();

  // Se o periferico assumiu o freio de XON/XOFF nesta configuracao.
  bool _freioXonXoffNoHardware = false;

  DncSerialConfig _cfg;
};
