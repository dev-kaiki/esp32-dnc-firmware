#pragma once

/*
  SMI ESP32 DNC Industrial - Configuracao de hardware
  Placa alvo: ESP32 DevKit V1 30 pinos

  Ligacao principal:
    ESP32 GPIO17 TX2 -> MAX3232 T1IN -> RS232 TX para CNC RX
    ESP32 GPIO16 RX2 <- MAX3232 R1OUT <- RS232 RX vindo CNC TX

  Observacao importante:
    Para usar RTS/CTS + DTR/DSR completos, o conversor RS232 precisa ter canais suficientes.
    Um modulo MAX3232 simples costuma ter 2 drivers e 2 receivers. TX/RX + RTS/CTS consome
    2 drivers e 2 receivers. DTR/DSR exige canais adicionais ou outro CI/modulo.
*/

// ---------- UART RS232 ----------
#define PIN_RS232_RX        16
#define PIN_RS232_TX        17
#define PIN_RS232_RTS       25   // Saida TTL para driver RS232: ESP32 RTS -> CNC CTS
#define PIN_RS232_CTS       26   // Entrada TTL de receiver RS232: CNC RTS/CTS -> ESP32 CTS
#define PIN_RS232_DTR       27   // Saida TTL para driver RS232: ESP32 DTR -> CNC DSR
#define PIN_RS232_DSR       33   // Entrada TTL de receiver RS232: CNC DTR/DSR -> ESP32 DSR

// Na maioria dos conversores MAX3232, sinais de controle RS232 assertados chegam como LOW no lado TTL.
#define FLOW_INPUT_ACTIVE_LEVEL LOW
#define FLOW_OUTPUT_ACTIVE_LEVEL LOW
#define FLOW_OUTPUT_INACTIVE_LEVEL HIGH

// ---------- microSD SPI ----------
#define PIN_SD_CS           5
#define PIN_SD_SCK          18
#define PIN_SD_MISO         19
#define PIN_SD_MOSI         23

// ---------- Sistema ----------
#define HOSTNAME            "smi-esp32-dnc"
#define DEFAULT_AP_SSID     "SMI_DNC_ESP32"
#define DEFAULT_AP_PASS     "12345678"
#define HTTP_PORT           80

// ---------- Buffers ----------
#define TX_RING_SIZE        8192
#define RX_RING_SIZE        8192
#define SD_READ_BLOCK       256
#define SD_WRITE_BLOCK      256
#define LOG_QUEUE_LENGTH    40
#define MAX_RECENT_LOGS     80

// ---------- Timeouts ----------
#define WIFI_RECONNECT_MS   10000
#define STATUS_UPDATE_MS    500
#define SERIAL_TASK_DELAY   1
#define STORAGE_IDLE_DELAY  20
#define DEFAULT_RECV_SILENCE_TIMEOUT_MS 3000

// ---------- Diretorios padrao no SD ----------
#define DIR_PROGRAMS        "/programs"
#define DIR_RECEIVED        "/received"
#define DIR_BACKUPS         "/backups"
#define DIR_LOGS            "/logs"
#define DIR_TMP             "/tmp"

// ---------- Login inicial ----------
// Trocar na primeira instalacao. HTTP puro nao protege senha na rede.
#define DEFAULT_ADMIN_USER  "admin"
#define DEFAULT_ADMIN_PASS  "smiadmin"
#define DEFAULT_OPER_USER   "operador"
#define DEFAULT_OPER_PASS   "operador"
#define DEFAULT_VIEW_USER   "viewer"
#define DEFAULT_VIEW_PASS   "viewer"
