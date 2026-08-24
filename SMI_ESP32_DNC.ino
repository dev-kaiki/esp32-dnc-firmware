/*
  SMI ESP32 DNC Industrial
  Hardware: ESP32 DevKit V1 30 pinos + MAX3232 + microSD
  IDE: Arduino IDE / Arduino-ESP32 core

  Arquitetura operacional:
      microSD -> Buffer Circular TX -> TaskSerial -> RS232 -> CNC
      CNC     -> RS232 -> TaskSerial -> Buffer Circular RX -> TaskDNC -> microSD

  Modos:
    - Enviar para memoria CNC
    - Receber da CNC
    - Executar Online / Drip Feed

  Observacao de engenharia:
    Este firmware e uma base industrial modular para bancada e evolucao de produto.
    Antes de usar em producao, validar eletricamente isolacao, aterramento, DB9, handshake real
    e protocolo de cada CNC.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <SD.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>

#include "Config.h"
#include "src/Types.h"
#include "src/CircularBuffer.h"
#include "src/Logger.h"
#include "src/Settings.h"
#include "src/SerialManager.h"
#include "src/StorageManager.h"
#include "src/WebPortal.h"

// ---------- Globais compartilhados ----------
CircularBuffer txRing;
CircularBuffer rxRing;
Settings settings;
StorageManager storage;
SerialManager serialManager;

QueueHandle_t gDncCommandQueue = nullptr;
SemaphoreHandle_t gStatusMutex = nullptr;
SemaphoreHandle_t gLogMutex = nullptr;

SystemStatus gStatus;
DncSerialConfig gSerialConfig;
NetworkConfig gNetworkConfig;

String gRecentLogs[MAX_RECENT_LOGS];
uint16_t gRecentLogHead = 0;

// Controle XON/XOFF: true = permitido transmitir. XOFF muda para false.
volatile bool gXonAllowed = true;

// ---------- Utilitarios de estado ----------
static void setState(DncState state) {
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  gStatus.state = state;
  xSemaphoreGive(gStatusMutex);
}

static void setError(const char* err) {
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  gStatus.state = DNC_ERROR;
  strncpy(gStatus.lastError, err, sizeof(gStatus.lastError) - 1);
  xSemaphoreGive(gStatusMutex);
  logEvent("ERRO", err);
}

static bool isCancelRequested() {
  bool v;
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  v = gStatus.cancelRequested;
  xSemaphoreGive(gStatusMutex);
  return v;
}

static bool isUserPaused() {
  bool v;
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  v = gStatus.userPaused;
  xSemaphoreGive(gStatusMutex);
  return v;
}

static void updateStatusFlags() {
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  gStatus.serialConnected = serialManager.isConnected();
  gStatus.sdOk = storage.isOk();
  gStatus.wifiOk = WiFi.isConnected();
  gStatus.apMode = WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA;
  gStatus.flowPaused = !gXonAllowed;
  gStatus.ctsOk = serialManager.ctsOk();
  gStatus.dsrOk = serialManager.dsrOk();
  if (gStatus.transferStartMs > 0) {
    uint32_t elapsed = (millis() - gStatus.transferStartMs) / 1000;
    if (elapsed > 0) gStatus.bytesPerSecond = gStatus.bytesSent / elapsed;
  }
  xSemaphoreGive(gStatusMutex);
}

static void addRecentLog(const LogMessage& msg) {
  char line[260];
  snprintf(line, sizeof(line), "[%10lu ms] [%s] %s", (unsigned long)msg.ms, msg.level, msg.text);
  xSemaphoreTake(gLogMutex, portMAX_DELAY);
  gRecentLogs[gRecentLogHead] = String(line);
  gRecentLogHead = (gRecentLogHead + 1) % MAX_RECENT_LOGS;
  xSemaphoreGive(gLogMutex);
}

static String autoReceiveName() {
  char buf[80];
  snprintf(buf, sizeof(buf), DIR_RECEIVED "/RX_%lu.nc", (unsigned long)millis());
  return String(buf);
}

// ---------- Watchdog ----------
static void setupWatchdogForCurrentTask() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_add(NULL);
#else
  esp_task_wdt_add(NULL);
#endif
}

static void setupWatchdogSystem() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t twdt_config = {};
  twdt_config.timeout_ms = 15000;
  twdt_config.idle_core_mask = (1 << portNUM_PROCESSORS) - 1;
  twdt_config.trigger_panic = true;
  esp_task_wdt_init(&twdt_config);
#else
  esp_task_wdt_init(15, true);
#endif
}

// ---------- Rede ----------
static void startNetwork() {
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA);

  if (gNetworkConfig.ssid.length()) {
    if (!gNetworkConfig.dhcp) {
      WiFi.config(gNetworkConfig.ip, gNetworkConfig.gateway, gNetworkConfig.subnet, gNetworkConfig.dns);
    }
    WiFi.begin(gNetworkConfig.ssid.c_str(), gNetworkConfig.password.c_str());
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
      delay(250);
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(DEFAULT_AP_SSID, DEFAULT_AP_PASS);
    logEventf("INFO", "AP iniciado: %s IP %s", DEFAULT_AP_SSID, WiFi.softAPIP().toString().c_str());
  } else {
    logEventf("INFO", "Wi-Fi conectado: %s IP %s", gNetworkConfig.ssid.c_str(), WiFi.localIP().toString().c_str());
  }
}

// ---------- TaskSerial: UART, RX/TX, fluxo ----------
void TaskSerial(void* pv) {
  setupWatchdogForCurrentTask();
  logEvent("INFO", "TaskSerial iniciada");

  uint32_t lastFlowLog = 0;
  bool previousCanTx = true;

  for (;;) {
    esp_task_wdt_reset();

    // Leitura da CNC: captura XON/XOFF e dados normais.
    int rb = serialManager.readByte();
    while (rb >= 0) {
      uint8_t b = (uint8_t)rb;
      if (gSerialConfig.flow == FLOW_XONXOFF && b == 0x13) { // XOFF
        gXonAllowed = false;
        setState(DNC_PAUSED_FLOW);
        logEvent("FLOW", "XOFF recebido - transmissao pausada pela CNC");
      } else if (gSerialConfig.flow == FLOW_XONXOFF && b == 0x11) { // XON
        gXonAllowed = true;
        logEvent("FLOW", "XON recebido - transmissao retomada pela CNC");
      } else {
        rxRing.push(b);
        xSemaphoreTake(gStatusMutex, portMAX_DELAY);
        gStatus.bytesReceived++;
        gStatus.lastByteMs = millis();
        xSemaphoreGive(gStatusMutex);
      }
      rb = serialManager.readByte();
    }

    // Escrita para CNC: respeita XON/XOFF, RTS/CTS, DTR/DSR ou sem fluxo.
    bool canTx = serialManager.canTransmit(gSerialConfig, gXonAllowed);
    if (canTx) {
      int tb = txRing.pop();
      if (tb >= 0) {
        serialManager.writeByte((uint8_t)tb);
        xSemaphoreTake(gStatusMutex, portMAX_DELAY);
        gStatus.bytesSent++;
        gStatus.lastByteMs = millis();
        xSemaphoreGive(gStatusMutex);
      }
    } else {
      if (previousCanTx && millis() - lastFlowLog > 500) {
        if (gSerialConfig.flow == FLOW_RTSCTS) logEvent("FLOW", "Aguardando CTS da CNC");
        if (gSerialConfig.flow == FLOW_DTRDSR) logEvent("FLOW", "Aguardando DSR da CNC");
        previousCanTx = false;
        lastFlowLog = millis();
      }
      vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (canTx && !previousCanTx) {
      previousCanTx = true;
      logEvent("FLOW", "Controle de fluxo liberado");
    }

    updateStatusFlags();
    vTaskDelay(pdMS_TO_TICKS(SERIAL_TASK_DELAY));
  }
}

// ---------- Operacoes DNC ----------
static bool waitTxDrain(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (txRing.available() > 0) {
    if (isCancelRequested()) return false;
    if (millis() - start > timeoutMs) return false;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  serialManager.flush();
  return true;
}

static void prepareTransferStatus(DncState state, const String& path, uint64_t fileSize) {
  txRing.clear();
  rxRing.clear();
  gXonAllowed = true;
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  gStatus.state = state;
  gStatus.cancelRequested = false;
  gStatus.userPaused = false;
  strncpy(gStatus.currentFile, path.c_str(), sizeof(gStatus.currentFile) - 1);
  gStatus.fileSize = fileSize;
  gStatus.bytesSent = 0;
  gStatus.bytesReceived = 0;
  gStatus.bytesPerSecond = 0;
  gStatus.transferStartMs = millis();
  gStatus.lastByteMs = millis();
  gStatus.lastError[0] = 0;
  xSemaphoreGive(gStatusMutex);
}

static void streamFileToCnc(const String& path, bool dripMode) {
  if (!storage.exists(path)) {
    setError("Arquivo nao encontrado no microSD");
    return;
  }

  uint64_t size = storage.fileSize(path);
  prepareTransferStatus(dripMode ? DNC_DRIP_FEED : DNC_SENDING, path, size);
  logEventf("INFO", "%s iniciado: %s (%llu bytes)", dripMode ? "Drip feed" : "Envio", path.c_str(), size);

  uint8_t block[SD_READ_BLOCK];
  uint64_t readTotal = 0;

  xSemaphoreTake(storage.mutex(), portMAX_DELAY);
  File f = SD.open(path, FILE_READ);
  if (!f || f.isDirectory()) {
    if (f) f.close();
    xSemaphoreGive(storage.mutex());
    setError("Falha ao abrir arquivo para leitura");
    return;
  }

  while (f.available()) {
    if (isCancelRequested()) {
      logEvent("WARN", "Transferencia cancelada pelo operador");
      break;
    }

    while (isUserPaused()) {
      setState(DNC_PAUSED_USER);
      vTaskDelay(pdMS_TO_TICKS(50));
      if (isCancelRequested()) break;
    }

    if (isCancelRequested()) break;
    setState(dripMode ? DNC_DRIP_FEED : DNC_SENDING);

    size_t n = f.read(block, sizeof(block));
    readTotal += n;
    size_t pushed = 0;
    while (pushed < n) {
      if (isCancelRequested()) break;
      if (isUserPaused()) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
      size_t p = 0;
      txRing.pushBlock(block + pushed, n - pushed, &p);
      pushed += p;
      if (p == 0) vTaskDelay(pdMS_TO_TICKS(2));
    }

    if ((readTotal % 4096) < SD_READ_BLOCK) {
      logEventf("INFO", "%llu/%llu bytes lidos do SD", readTotal, size);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  f.close();
  xSemaphoreGive(storage.mutex());

  if (!isCancelRequested()) {
    if (!waitTxDrain(gSerialConfig.timeoutMs + 8000)) {
      setError("Timeout aguardando drenagem final do buffer TX");
      return;
    }
    logEvent("OK", dripMode ? "Drip feed concluido" : "Envio concluido");
    setState(DNC_CONNECTED);
  } else {
    txRing.clear();
    setState(DNC_CONNECTED);
  }
}

static void receiveFromCnc(const String& requestedPath) {
  String path = requestedPath.length() > 1 ? requestedPath : autoReceiveName();
  prepareTransferStatus(DNC_RECEIVING, path, 0);
  logEventf("INFO", "Recepcao iniciada: %s", path.c_str());

  uint8_t block[SD_WRITE_BLOCK];
  uint32_t lastRx = millis();
  uint64_t written = 0;

  xSemaphoreTake(storage.mutex(), portMAX_DELAY);
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    xSemaphoreGive(storage.mutex());
    setError("Falha ao abrir arquivo de recepcao");
    return;
  }

  for (;;) {
    if (isCancelRequested()) {
      logEvent("WARN", "Recepcao cancelada pelo operador");
      break;
    }

    size_t n = rxRing.popBlock(block, sizeof(block));
    if (n > 0) {
      f.write(block, n);
      written += n;
      lastRx = millis();
      xSemaphoreTake(gStatusMutex, portMAX_DELAY);
      gStatus.fileSize = written;
      xSemaphoreGive(gStatusMutex);
    } else {
      // Encerramento automatico por silencio apos receber algum dado.
      if (written > 0 && millis() - lastRx > gSerialConfig.timeoutMs) break;
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  f.flush();
  f.close();
  xSemaphoreGive(storage.mutex());

  logEventf("OK", "Recepcao finalizada: %s (%llu bytes)", path.c_str(), written);
  setState(DNC_CONNECTED);
}

// ---------- TaskDNC: orquestra comandos ----------
void TaskDNC(void* pv) {
  setupWatchdogForCurrentTask();
  logEvent("INFO", "TaskDNC iniciada");
  DncCommand cmd;

  for (;;) {
    esp_task_wdt_reset();
    if (xQueueReceive(gDncCommandQueue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
      switch (cmd.type) {
        case CMD_CONNECT:
          serialManager.connectLines();
          setState(DNC_CONNECTED);
          logEvent("INFO", "RS232 conectado/linhas habilitadas");
          break;

        case CMD_DISCONNECT:
          txRing.clear();
          rxRing.clear();
          serialManager.disconnectLines();
          setState(DNC_IDLE);
          logEvent("INFO", "RS232 desconectado/linhas desabilitadas");
          break;

        case CMD_APPLY_SERIAL:
          serialManager.applyConfig(gSerialConfig);
          setState(DNC_CONNECTED);
          logEventf("INFO", "Serial aplicado: %lu %u%s%u %s", (unsigned long)gSerialConfig.baudRate, gSerialConfig.dataBits, parityToText(gSerialConfig.parity), gSerialConfig.stopBits, flowToText(gSerialConfig.flow));
          break;

        case CMD_SEND_FILE:
          streamFileToCnc(String(cmd.path), false);
          break;

        case CMD_DRIP_FILE:
          streamFileToCnc(String(cmd.path), true);
          break;

        case CMD_RECEIVE_FILE:
          receiveFromCnc(String(cmd.path));
          break;

        case CMD_PAUSE:
          xSemaphoreTake(gStatusMutex, portMAX_DELAY);
          gStatus.userPaused = true;
          gStatus.state = DNC_PAUSED_USER;
          xSemaphoreGive(gStatusMutex);
          logEvent("INFO", "Pausa solicitada pelo operador");
          break;

        case CMD_RESUME:
          xSemaphoreTake(gStatusMutex, portMAX_DELAY);
          gStatus.userPaused = false;
          if (gStatus.currentFile[0]) gStatus.state = DNC_CONNECTED;
          xSemaphoreGive(gStatusMutex);
          logEvent("INFO", "Retomada solicitada pelo operador");
          break;

        case CMD_CANCEL:
          xSemaphoreTake(gStatusMutex, portMAX_DELAY);
          gStatus.cancelRequested = true;
          gStatus.userPaused = false;
          gStatus.state = DNC_CANCELING;
          xSemaphoreGive(gStatusMutex);
          txRing.clear();
          logEvent("WARN", "Cancelamento solicitado");
          break;

        default:
          break;
      }
    }
  }
}

// ---------- TaskStorage: diagnostico e manutencao do SD ----------
void TaskStorage(void* pv) {
  setupWatchdogForCurrentTask();
  logEvent("INFO", "TaskStorage iniciada");
  for (;;) {
    esp_task_wdt_reset();
    if (!storage.isOk()) {
      bool ok = storage.begin();
      if (ok) logEvent("OK", "microSD recuperado");
    }
    xSemaphoreTake(gStatusMutex, portMAX_DELAY);
    gStatus.sdOk = storage.isOk();
    xSemaphoreGive(gStatusMutex);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ---------- TaskNetwork: reconexao Wi-Fi ----------
void TaskNetwork(void* pv) {
  setupWatchdogForCurrentTask();
  logEvent("INFO", "TaskNetwork iniciada");
  uint32_t lastTry = 0;
  for (;;) {
    esp_task_wdt_reset();
    if (WiFi.getMode() == WIFI_STA && WiFi.status() != WL_CONNECTED && millis() - lastTry > WIFI_RECONNECT_MS) {
      lastTry = millis();
      WiFi.disconnect();
      WiFi.begin(gNetworkConfig.ssid.c_str(), gNetworkConfig.password.c_str());
      logEvent("WARN", "Tentando reconectar Wi-Fi");
    }
    updateStatusFlags();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ---------- TaskWebServer ----------
void TaskWebServer(void* pv) {
  setupWatchdogForCurrentTask();
  logEvent("INFO", "TaskWebServer iniciada");
  for (;;) {
    esp_task_wdt_reset();
    webPortal.loop();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ---------- TaskLogger ----------
void TaskLogger(void* pv) {
  setupWatchdogForCurrentTask();
  LogMessage msg;
  for (;;) {
    esp_task_wdt_reset();
    if (xQueueReceive(gLogQueue, &msg, pdMS_TO_TICKS(250)) == pdTRUE) {
      addRecentLog(msg);
      Serial.printf("[%lu ms] [%s] %s\n", (unsigned long)msg.ms, msg.level, msg.text);

      if (storage.isOk()) {
        xSemaphoreTake(storage.mutex(), portMAX_DELAY);
        File f = SD.open(DIR_LOGS "/dnc.log", FILE_APPEND);
        if (f) {
          f.printf("[%lu ms] [%s] %s\n", (unsigned long)msg.ms, msg.level, msg.text);
          f.close();
        }
        xSemaphoreGive(storage.mutex());
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nSMI ESP32 DNC Industrial iniciando...");

  gStatusMutex = xSemaphoreCreateMutex();
  gLogMutex = xSemaphoreCreateMutex();
  gLogQueue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(LogMessage));
  gDncCommandQueue = xQueueCreate(12, sizeof(DncCommand));

  setupWatchdogSystem();
  setupWatchdogForCurrentTask();

  if (!txRing.begin(TX_RING_SIZE) || !rxRing.begin(RX_RING_SIZE)) {
    Serial.println("ERRO: falha ao alocar buffers circulares");
    while (true) delay(1000);
  }

  settings.begin();
  gSerialConfig = settings.loadSerial();
  gNetworkConfig = settings.loadNetwork();

  bool sd = storage.begin();
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  gStatus.sdOk = sd;
  xSemaphoreGive(gStatusMutex);
  if (!sd) Serial.println("WARN: microSD nao inicializou");

  serialManager.begin(gSerialConfig);
  serialManager.disconnectLines();

  startNetwork();
  webPortal.begin();

  xTaskCreatePinnedToCore(TaskLogger,    "TaskLogger",    4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(TaskSerial,    "TaskSerial",    4096, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(TaskDNC,       "TaskDNC",       6144, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(TaskStorage,   "TaskStorage",   4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(TaskNetwork,   "TaskNetwork",   4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(TaskWebServer, "TaskWebServer", 8192, nullptr, 2, nullptr, 0);

  logEvent("OK", "Sistema inicializado");
  setState(DNC_IDLE);
}

void loop() {
  esp_task_wdt_reset();
  updateStatusFlags();
  vTaskDelay(pdMS_TO_TICKS(1000));
}
