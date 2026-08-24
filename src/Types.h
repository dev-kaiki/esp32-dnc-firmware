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
};

struct NetworkConfig {
  String ssid;
  String password;
  bool dhcp = true;
  IPAddress ip;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress dns;
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
  DNC_ERROR
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
