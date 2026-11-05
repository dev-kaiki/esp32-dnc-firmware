#pragma once
#include <Arduino.h>

// ---------- Serial ----------
enum ParityMode : uint8_t {
  PARITY_NONE = 0,
  PARITY_EVEN = 1,
  PARITY_ODD  = 2
};

enum FlowControlMode : uint8_t {
  FLOW_NONE   = 0,
  FLOW_XONXOFF = 1,
  FLOW_RTSCTS = 2,
  FLOW_DTRDSR = 3
};

struct DncSerialConfig {
  uint32_t baudRate = 9600;
  uint8_t dataBits = 7;
  ParityMode parity = PARITY_EVEN;
  uint8_t stopBits = 1;
  uint32_t timeoutMs = 3000;
  FlowControlMode flow = FLOW_XONXOFF;

  // AJUSTES DO MOTOR QUE VEM DO APLICATIVO.
  //
  // Estes campos faltavam, e a falta era invisivel: o aplicativo mandava os
  // valores, o nucleo do protocolo lia todos certinho, e a traducao para esta
  // struct simplesmente os deixava cair. O motor seguia com os padroes de
  // fabrica achando que ninguem tinha configurado nada.
  //
  // Custou caro no limite de envio: o operador escolhia 120 B/s, depois 60, e o
  // resultado era identico -- porque o numero morria aqui e o motor nunca via
  // limite nenhum. Ver a nota em traduzirConfigDoMotor.
  uint32_t taxaManualBps = 0;      // 0 = sem limite
  uint32_t chunkInicial  = 0;      // 0 = usar o padrao do motor
  uint32_t chunkMin      = 0;
  uint32_t chunkMax      = 0;
  uint32_t holdSemConsumoMs = 0;
  uint32_t debounceMs    = 0;
};

struct NetworkConfig {
  // NOME PROPRIO DESTE EQUIPAMENTO, um por caixa.
  //
  // Antes era uma constante no codigo, igual em toda a frota. Numa fabrica com
  // varias caixas isso significava:
  //   - a lista de aparelhos do roteador com N nomes identicos, tornando
  //     "descobrir o IP da caixa" pior do que ja era;
  //   - o OTA mostrando todas com o mesmo nome, ou seja, gravar firmware na
  //     caixa da MAQUINA ERRADA sem perceber;
  //   - mDNS impossivel: duas caixas anunciando o mesmo nome colidem.
  //
  // Mesma falha da senha de administrador: identidade igual em todo aparelho.
  String nomeDoEquipamento;

  String ssid;
  String password;
  // Senha da atualizacao de firmware pelo ar (OTA). Vazia = OTA desligado, de
  // proposito: caixa em rede de fabrica sem senha e regravavel por qualquer um.
  String otaPassword;
  bool dhcp = true;
  IPAddress ip;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress dns;
};

// ---------- Acesso ao portal ----------
//
// AS SENHAS VIVEM NA MEMORIA DO EQUIPAMENTO, uma por aparelho.
//
// Antes elas eram constantes no codigo, iguais em todo equipamento que saisse.
// Quem descobrisse a senha de um entraria em todos -- e, pela aba Rede,
// chegaria na senha de atualizacao de firmware. Numa rede de fabrica isso e
// acesso de regravacao ao equipamento que comanda a CNC.
//
// Os nomes de usuario continuam fixos de proposito: sao tres papeis, nao tres
// pessoas, e nome de usuario nao e segredo.
struct AccessConfig {
  String adminPass;
  String operPass;
  String viewPass;
};

// ---------- Estado do DNC ----------
enum DncState : uint8_t {
  DNC_IDLE = 0,
  DNC_CONNECTED,
  DNC_SENDING,
  DNC_RECEIVING,
  DNC_DRIP_FEED,
  DNC_PAUSED_USER,
  DNC_PAUSED_FLOW,
  DNC_CANCELING,
  DNC_ERROR,

  // Programa inteiro entregue a porta; a maquina pode ainda estar executando.
  //
  // ACRESCENTADO NO FIM DE PROPOSITO: o portal e o aplicativo mandam o estado
  // como numero cru, entao renumerar os anteriores quebraria quem ja esta
  // instalado.
  //
  // Este estado existe porque "terminou de sair pela serial" NAO e "a maquina
  // terminou de usinar". Fechar a comunicacao no primeiro pode cortar o final
  // do programa. Aqui o equipamento para e espera o operador confirmar.
  DNC_AWAIT_FINISH,

  // A drenagem final parou de avancar. A PORTA CONTINUA ABERTA -- isto e uma
  // pergunta ao operador, nao um encerramento.
  DNC_DRAIN_STALLED,

  // A MAQUINA TERMINOU DE MANDAR E O ARQUIVO ESTA GRAVADO NO CARTAO.
  //
  // POR QUE ISTO PRECISOU EXISTIR: a recepcao terminava com DNC_CONNECTED, que
  // e um dos estados marcados como "nada a publicar". O aplicativo nunca era
  // avisado de que acabou -- a tela ficava esperando para sempre e o operador
  // tinha que cancelar na mao. Com o download automatico isso ficou pior: sem
  // o aviso, o programa recebido nunca era buscado e nao chegava ao tablete.
  //
  // NAO conta como ocupado: a operacao com a maquina acabou de verdade.
  DNC_RECEIVE_DONE,

  // A MAQUINA PAROU DE MANDAR, MAS PODE NAO TER ACABADO. A PORTA FICA ABERTA.
  //
  // Parametros de Siemens saem em blocos com pausas longas no meio -- a maquina
  // chega a ser desligada por um tempo durante o processo. O encerramento
  // automatico por silencio fechava a porta na primeira pausa e o resto dos
  // parametros se perdia, com "sucesso" na tela.
  //
  // Mesma logica do DNC_AWAIT_FINISH no envio: silencio NAO e fim. Quem decide
  // se acabou e o operador, com a porta ainda aberta.
  DNC_RECEIVE_IDLE
};

enum DncCommandType : uint8_t {
  CMD_NONE = 0,
  CMD_CONNECT,
  CMD_DISCONNECT,
  CMD_SEND_FILE,
  CMD_RECEIVE_FILE,
  CMD_DRIP_FILE,
  CMD_PAUSE,
  CMD_RESUME,
  CMD_CANCEL,
  CMD_APPLY_SERIAL
};

struct DncCommand {
  DncCommandType type = CMD_NONE;
  char path[180] = {0};
  char path2[180] = {0};
};

struct SystemStatus {
  DncState state = DNC_IDLE;
  bool serialConnected = false;
  bool sdOk = false;
  bool wifiOk = false;
  bool apMode = false;
  bool flowPaused = false;
  bool ctsOk = true;
  bool dsrOk = true;
  bool cancelRequested = false;
  bool userPaused = false;
  char currentFile[180] = {0};
  uint64_t fileSize = 0;
  uint64_t bytesSent = 0;
  uint64_t bytesReceived = 0;
  uint32_t transferStartMs = 0;
  uint32_t lastByteMs = 0;
  uint32_t bytesPerSecond = 0;
  char lastError[160] = {0};
};

struct LogMessage {
  char level[8];
  char text[180];
  uint32_t ms;
};

static inline const char* stateToText(DncState s) {
  switch (s) {
    case DNC_IDLE: return "Parado";
    case DNC_CONNECTED: return "Conectado";
    case DNC_SENDING: return "Enviando para memoria";
    case DNC_RECEIVING: return "Recebendo";
    case DNC_DRIP_FEED: return "Executando Online / Drip Feed";
    case DNC_PAUSED_USER: return "Pausado pelo operador";
    case DNC_PAUSED_FLOW: return "Pausado por controle de fluxo";
    case DNC_CANCELING: return "Cancelando";
    case DNC_ERROR: return "Erro";
    case DNC_AWAIT_FINISH: return "Transferido - aguardando confirmacao do operador";
    case DNC_DRAIN_STALLED: return "A maquina parou de consumir - porta aberta, aguardando";
    case DNC_RECEIVE_DONE: return "Programa recebido e gravado no cartao";
    default: return "Desconhecido";
  }
}

static inline const char* parityToText(ParityMode p) {
  switch (p) {
    case PARITY_NONE: return "NONE";
    case PARITY_EVEN: return "EVEN";
    case PARITY_ODD: return "ODD";
    default: return "NONE";
  }
}

static inline const char* flowToText(FlowControlMode f) {
  switch (f) {
    case FLOW_NONE: return "NONE";
    case FLOW_XONXOFF: return "XON/XOFF";
    case FLOW_RTSCTS: return "RTS/CTS";
    case FLOW_DTRDSR: return "DTR/DSR";
    default: return "NONE";
  }
}
