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
#include <algorithm>
#include <WiFi.h>
#include <SD.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>
#include <esp_ota_ops.h>

#include "Config.h"
#include "src/Types.h"
#include "src/CircularBuffer.h"
#include "src/Logger.h"
#include "src/Settings.h"
#include "src/SerialManager.h"
#include "src/StorageManager.h"
#include "src/WebPortal.h"
#include "src/OtaUpdater.h"
#include "src/DncMotorCore.h"
#include "src/DncProtocolServer.h"
#include "src/DescobertaLan.h"

// ---------- Motor de envio ----------
//
// Mesmo codigo do aplicativo Android, portado para C++ puro e conferido contra
// o original passo a passo (ferramentas/testes/teste_motor_esp32.cpp). E ele
// que decide QUANTO e QUANDO escrever -- a diferenca entre alimentar uma CNC e
// despejar um arquivo na serial.
smi::dnc::motor::ConfigDoMotor      gConfigMotor;
smi::dnc::motor::AgendadorDeEnvio   gAgendador;
smi::dnc::motor::ControladorDeFluxo gFluxo;
smi::dnc::motor::VigiaDeConsumo     gVigia;

// A sessao de envio. Quem manda enquanto ela esta ativa.
smi::dnc::motor::MotorDeSessao      gSessao;

// Servidor do protocolo binario com o aplicativo Android.
//
// A LOGICA NAO ESTA AQUI: ela mora no DncProtocolCore, que e C++ puro e tem 38
// testes rodando fora do aparelho. Este objeto e a casca Arduino.
smi::dnc::esp::DncProtocolServer gProtocolo;

// Descoberta na rede: e como o aplicativo acha esta caixa sem ninguem digitar
// endereco. Ver a nota no cabecalho de DescobertaLan.h.
smi::dnc::esp::DescobertaLan gDescoberta;

// DUAS TAREFAS MEXEM NO MOTOR, entao ele precisa de tranca.
//
// A TaskDNC abre e fecha a sessao e responde ao operador; a TaskSerial pergunta
// quanto escrever e informa o que escreveu. Sem tranca, uma leitura de estado no
// meio de uma transicao devolveria decisao tomada sobre dado pela metade -- e
// aqui decisao errada significa byte a mais ou a menos chegando na maquina.
SemaphoreHandle_t gMotorMutex = nullptr;

static inline void travarMotor()   { if (gMotorMutex) xSemaphoreTake(gMotorMutex, portMAX_DELAY); }
static inline void destravarMotor(){ if (gMotorMutex) xSemaphoreGive(gMotorMutex); }

// Quanto do arquivo ainda NAO foi empurrado para a fila de saida.
//
// POR QUE PRECISA EXISTIR: a TaskSerial so enxerga a fila. Se a leitura do
// cartao ficar um instante atras, a fila esvazia e a TaskSerial concluiria que o
// arquivo acabou -- comecando a drenagem final no meio do programa e encerrando
// o envio cedo. Somando este contador, "acabou" so vale quando acabou mesmo.
//
// 32 bits, e nao 64, porque a escrita alinhada de 32 bits e atomica no ESP32:
// da para ler daqui sem tranca. Programa de CNC nao chega perto de 4 GB.
volatile uint32_t gRestanteNoArquivo = 0;

// ---------- Globais compartilhados ----------
CircularBuffer txRing;
CircularBuffer rxRing;
Settings settings;
StorageManager storage;
SerialManager serialManager;
smi::dnc::esp::OtaUpdater gOta;

QueueHandle_t gDncCommandQueue = nullptr;
SemaphoreHandle_t gStatusMutex = nullptr;
SemaphoreHandle_t gLogMutex = nullptr;
SemaphoreHandle_t gConfigMutex = nullptr;

SystemStatus gStatus;
DncSerialConfig gSerialConfig;
NetworkConfig gNetworkConfig;
AccessConfig gAccessConfig;

// Contador atomico de perdas no caminho CNC -> anel RX. A TaskSerial escreve e
// a TaskDNC compara com o valor do inicio da recepcao.
volatile uint32_t gRxOverflowCount = 0;

String gRecentLogs[MAX_RECENT_LOGS];
uint16_t gRecentLogHead = 0;

// Controle XON/XOFF: true = permitido transmitir. XOFF muda para false.
// QUAL TAREFA TRAVOU. SOBREVIVE AO REINICIO.
//
// O motivo do reinicio ja dizia "WATCHDOG DE TAREFA", mas nao QUAL -- e esse
// nome so sai pela serial USB, que numa caixa instalada em maquina ninguem
// alcanca. Sem ele, cada travamento vira adivinhacao entre seis tarefas.
//
// RTC_NOINIT_ATTR fica numa area de memoria que o reset NAO zera. Cada tarefa
// carimba aqui a hora da ultima vez que alimentou o cao; depois do reinicio,
// a que estiver com a marca mais ANTIGA e a culpada.
//
// A palavra magica separa "dados validos de um travamento" de lixo de memoria
// numa partida a frio, onde esta area vem com qualquer coisa.
#define MARCA_VALIDA 0x5A1DE2C0u
RTC_NOINIT_ATTR uint32_t gMarcaValida;
RTC_NOINIT_ATTR uint32_t gBatidaDaTarefa[8];
RTC_NOINIT_ATTR char     gNomeDaTarefa[8][12];

// Cada tarefa tem seu lugar fixo, para o indice nao depender de ordem de
// criacao (que muda quando alguem mexe no setup).
enum IndiceDeTarefa : uint8_t {
  TAREFA_SERIAL = 0, TAREFA_DNC, TAREFA_STORAGE,
  TAREFA_NETWORK, TAREFA_WEB, TAREFA_LOGGER, TAREFA_MAX
};

static void baterPonto(uint8_t qual) {
  if (qual < TAREFA_MAX) gBatidaDaTarefa[qual] = millis();
}

String gMotivoDoReinicio;
// Preenchida quando a configuracao salva nao passa na validacao do boot.
String gConfigSerialInvalida;

// FOTOGRAFIA DAS MARCAS, TIRADA ANTES DE QUALQUER TAREFA NASCER.
//
// O relatorio lia gBatidaDaTarefa[] la embaixo do setup -- mas as tarefas ja
// tinham sido criadas e JA TINHAM CARIMBADO POR CIMA. Resultado: cinco das seis
// marcas mostravam o tempo do boot NOVO (~4615 ms) e o relatorio acusava a
// tarefa errada. A informacao se perdia justamente no caso que ela existe para
// explicar.
//
// Agora a copia e feita na primeira linha do setup, quando so existe a tarefa
// de arranque e ninguem mais pode escrever ali.
static uint32_t gBatidaNoBootAnterior[8];
static char     gNomeNoBootAnterior[8][12];
static bool     gTinhaMarcasValidas = false;

// TROCA DE REDE PEDIDA PELO APLICATIVO, APLICADA DEPOIS.
//
// Entrar na rede nova derruba o ponto de acesso de socorro e, com ele, a
// conexao por onde o pedido chegou. Aplicar na hora faria o aplicativo esperar
// para sempre uma resposta que morreu no meio do caminho -- e o operador veria
// "falhou" num cadastro que deu certo. Entao o protocolo responde primeiro e a
// TaskNetwork troca na volta seguinte.
volatile bool gTrocaDeRedePedida = false;
String gRedeNovaSsid;
String gRedeNovaSenha;

// Chamada pelo SerialManager ao configurar a porta. Existe porque o
// SerialManager nao enxerga o logEvent daqui, e esta informacao PRECISA chegar
// ao historico: numa caixa instalada em maquina ninguem tem acesso ao cabo USB.
void smiLogFreio(bool noHardware, int codigo) {
  logEventf(noHardware ? "OK" : "WARN",
            "Freio XON/XOFF no %s (uart_set_sw_flow_ctrl -> %d)",
            noHardware ? "HARDWARE - para em 1 caractere"
                       : "SOFTWARE - chega tarde para Fanuc",
            codigo);
}
volatile bool gXonAllowed = true;


// --- Resposta do operador a uma pausa da maquina na RECEPCAO ----------------
//
// Duas bandeiras, lidas pelo laco de receiveFromCnc. Volatile porque quem
// escreve e a tarefa do protocolo e quem le e a TaskDNC.
volatile bool gRecepcaoContinuar = false;
volatile bool gRecepcaoFinalizar = false;

// Declarada aqui porque as lambdas do protocolo, mais acima no arquivo, a usam
// antes da definicao. O .ino gera prototipos sozinho para funcoes normais, mas
// nao para as marcadas static.
static DncState dncEstadoAtual();

// Copias protegidas evitam ler um String ou uma configuracao pela metade
// enquanto o portal salva novos valores em outra tarefa.
DncSerialConfig copiarConfigSerial() {
  DncSerialConfig c;
  if (gConfigMutex) xSemaphoreTake(gConfigMutex, portMAX_DELAY);
  c = gSerialConfig;
  if (gConfigMutex) xSemaphoreGive(gConfigMutex);
  return c;
}

NetworkConfig copiarConfigRede() {
  NetworkConfig c;
  if (gConfigMutex) xSemaphoreTake(gConfigMutex, portMAX_DELAY);
  c = gNetworkConfig;
  if (gConfigMutex) xSemaphoreGive(gConfigMutex);
  return c;
}

AccessConfig copiarConfigAcesso() {
  AccessConfig c;
  if (gConfigMutex) xSemaphoreTake(gConfigMutex, portMAX_DELAY);
  c = gAccessConfig;
  if (gConfigMutex) xSemaphoreGive(gConfigMutex);
  return c;
}

void atualizarConfigSerial(const DncSerialConfig& c) {
  if (gConfigMutex) xSemaphoreTake(gConfigMutex, portMAX_DELAY);
  gSerialConfig = c;
  if (gConfigMutex) xSemaphoreGive(gConfigMutex);
}

void atualizarConfigRede(const NetworkConfig& c) {
  if (gConfigMutex) xSemaphoreTake(gConfigMutex, portMAX_DELAY);
  gNetworkConfig = c;
  if (gConfigMutex) xSemaphoreGive(gConfigMutex);
}

void atualizarConfigAcesso(const AccessConfig& c) {
  if (gConfigMutex) xSemaphoreTake(gConfigMutex, portMAX_DELAY);
  gAccessConfig = c;
  if (gConfigMutex) xSemaphoreGive(gConfigMutex);
}

bool validarConfigSerial(const DncSerialConfig& c, String& erro) {
  if (c.baudRate < 50 || c.baudRate > 921600) {
    erro = "baud rate fora da faixa 50..921600"; return false;
  }
  if (c.dataBits < 5 || c.dataBits > 8) {
    erro = "data bits deve ser 5, 6, 7 ou 8"; return false;
  }
  if ((uint8_t)c.parity > (uint8_t)PARITY_ODD) {
    erro = "paridade invalida"; return false;
  }
  if (c.stopBits != 1 && c.stopBits != 2) {
    erro = "stop bits deve ser 1 ou 2"; return false;
  }
  if ((uint8_t)c.flow > (uint8_t)FLOW_DTRDSR) {
    erro = "controle de fluxo invalido"; return false;
  }
  if (c.timeoutMs < 100 || c.timeoutMs > 600000) {
    erro = "timeout fora da faixa 100..600000 ms"; return false;
  }
  if (c.chunkInicial > TX_RING_SIZE || c.chunkMin > TX_RING_SIZE
      || c.chunkMax > TX_RING_SIZE) {
    erro = "tamanho de bloco maior que o buffer TX"; return false;
  }
  if (c.chunkMin && c.chunkMax && c.chunkMin > c.chunkMax) {
    erro = "bloco minimo maior que o maximo"; return false;
  }
  return true;
}

// ---------- Utilitarios de estado ----------
static void setState(DncState state) {
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  if (state == DNC_SENDING || state == DNC_DRIP_FEED || state == DNC_RECEIVING) {
    gStatus.resumeState = state;
  }
  // Uma mudanca de fluxo nao deve esconder a pausa explicita do operador.
  if (!(state == DNC_PAUSED_FLOW && gStatus.userPaused)) gStatus.state = state;
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

bool enfileirarComandoDnc(const DncCommand& cmd, uint32_t esperaMs) {
  const bool ok = gDncCommandQueue
               && xQueueSend(gDncCommandQueue, &cmd, pdMS_TO_TICKS(esperaMs)) == pdTRUE;
  if (!ok && gStatusMutex) {
    xSemaphoreTake(gStatusMutex, portMAX_DELAY);
    ++gStatus.commandQueueFull;
    xSemaphoreGive(gStatusMutex);
    logEvent("ERRO", "Fila de comandos DNC cheia");
  }
  return ok;
}

void solicitarPausaDnc(const char* origem) {
  bool aceitou = false;
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  const DncState s = gStatus.state;
  if (s == DNC_SENDING || s == DNC_DRIP_FEED || s == DNC_PAUSED_FLOW) {
    if (s != DNC_PAUSED_FLOW) gStatus.resumeState = s;
    gStatus.userPaused = true;
    gStatus.state = DNC_PAUSED_USER;
    aceitou = true;
  }
  xSemaphoreGive(gStatusMutex);
  logEventf(aceitou ? "INFO" : "WARN", "Pausa %s (%s)",
            aceitou ? "solicitada" : "ignorada: nenhuma transferencia ativa",
            origem ? origem : "origem desconhecida");
}

void solicitarRetomadaDnc(const char* origem) {
  bool aceitou = false;
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  if (gStatus.userPaused) {
    gStatus.userPaused = false;
    DncState proximo = gStatus.resumeState;
    if ((proximo == DNC_SENDING || proximo == DNC_DRIP_FEED) && !gXonAllowed) {
      proximo = DNC_PAUSED_FLOW;
    }
    if (proximo != DNC_SENDING && proximo != DNC_DRIP_FEED
        && proximo != DNC_RECEIVING && proximo != DNC_PAUSED_FLOW) {
      proximo = gStatus.serialConnected ? DNC_CONNECTED : DNC_IDLE;
    }
    gStatus.state = proximo;
    aceitou = true;
  }
  xSemaphoreGive(gStatusMutex);
  logEventf(aceitou ? "INFO" : "WARN", "Retomada %s (%s)",
            aceitou ? "solicitada" : "ignorada: equipamento nao estava pausado",
            origem ? origem : "origem desconhecida");
}

static void retomarEstadoAposFluxo() {
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  if (!gStatus.userPaused && gStatus.state == DNC_PAUSED_FLOW) {
    gStatus.state = gStatus.resumeState;
  }
  xSemaphoreGive(gStatusMutex);
}

void solicitarCancelamentoDnc(const char* origem) {
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  gStatus.cancelRequested = true;
  gStatus.userPaused = false;
  gStatus.state = DNC_CANCELING;
  xSemaphoreGive(gStatusMutex);

  txRing.clear();
  const size_t descartados = serialManager.descartarSaidaPendente();
  if (descartados > 0) {
    xSemaphoreTake(gStatusMutex, portMAX_DELAY);
    gStatus.uartDiscardedOnCancel += (uint32_t)descartados;
    xSemaphoreGive(gStatusMutex);
    logEventf("WARN", "Cancelamento: %u B presos na UART foram descartados",
              (unsigned)descartados);
  }
  logEventf("WARN", "Cancelamento solicitado (%s)",
            origem ? origem : "origem desconhecida");
}

// Traduz a configuracao da CNC para a configuracao do motor.
//
// O ponto delicado sao os dois ultimos campos. `interpretarXonXoff` e
// `exigirCtsAltoParaEnviar` significam "NOS tratamos isso", nao "isso esta
// ligado" -- e e essa distincao que decide qual dos tres caminhos de calculo do
// agendador vale. Aqui o firmware sempre trata por conta propria (nao usamos o
// fluxo por hardware do driver da UART), entao o modo do motor acompanha
// diretamente o modo escolhido para a maquina.
static smi::dnc::motor::ConfigDoMotor traduzirConfigDoMotor(const DncSerialConfig& c) {
  namespace M = smi::dnc::motor;
  M::ConfigDoMotor m;
  m.baudRate     = static_cast<int32_t>(c.baudRate);
  m.bitsDeDados  = c.dataBits;
  m.paridade     = (c.parity == PARITY_EVEN) ? M::Paridade::Par
                 : (c.parity == PARITY_ODD)  ? M::Paridade::Impar
                                             : M::Paridade::Nenhuma;
  m.bitsDeParada = c.stopBits;

  // O driver nao faz fluxo por conta: quem interpreta somos nos.
  m.controleDeFluxo         = M::ControleDeFluxo::Nenhum;
  m.interpretarXonXoff      = (c.flow == FLOW_XONXOFF);
  m.exigirCtsAltoParaEnviar = (c.flow == FLOW_RTSCTS);

  // NAO exigir XON inicial: muitas CNCs nao mandam um ao abrir a sessao, e
  // esperar por ele significaria nunca enviar o primeiro byte.
  m.exigirXonParaEnviar = false;

  // QUANTO PODE FICAR ESPERANDO NA FILA DA UART -- NAO E O TAMANHO DO BUFFER.
  //
  // Este numero decide quantos caracteres a maquina ainda vai receber DEPOIS de
  // mandar XOFF. Tudo o que ja foi entregue a UART sai no fio, e o XOFF nao
  // cancela o que esta na fila.
  //
  // Estava em UART_TX_BUFFER (1024), o que no modo XON/XOFF vira uma marca
  // d'agua de 512 bytes. A Mach9 absorve, porque tem buffer grande. Uma Fanuc
  // tem buffer pequeno: recebe meio kilobyte depois de ter pedido para parar,
  // transborda, e alarma COMMUNICATION ERROR. O envio pela mesma maquina com o
  // conversor USB funciona, e o conversor usa 256 -- metade.
  //
  // Entao aqui vai o valor QUE JA ESTA PROVADO EM CAMPO, e nao um palpite: o
  // mesmo do caminho que funciona. O buffer da UART continua com 1024 de folga;
  // o que muda e o quanto o motor se adianta.
  // COM XON/XOFF A JANELA TEM QUE SER MUITO MENOR.
  //
  // Este numero decide quantos caracteres a maquina ainda recebe DEPOIS de
  // mandar XOFF -- o que ja esta na fila da UART sai no fio de qualquer jeito.
  //
  // A Siemens acusou "Error COM1: Errorcode = 0001; overrun buffer" carregando
  // NCK com 256 de janela. Faz sentido: carga de parametro grava em memoria nao
  // volatil e a maquina PARA de consumir enquanto grava, com o buffer dela ja
  // cheio. Duzentos e cinquenta e seis caracteres depois do "pare" transbordam.
  //
  // Sessenta e quatro e ~66 ms de fio a 9600 -- perto do que um controle espera
  // como tempo de reacao. Custa pouco em vazao porque a 9600 quem manda no
  // ritmo e o fio, nao o nosso adiantamento.
  // LIMITE DE VELOCIDADE PEDIDO PELO OPERADOR.
  //
  // Existe para a maquina que NAO FREIA sozinha: a Siemens carregando NCK grava
  // dado de maquina em memoria nao volatil e para de consumir, e se o handshake
  // dela nao estiver atuando nada nos avisa. Alimentar num ritmo fixo abaixo do
  // que ela aguenta resolve sem depender de sinal nenhum.
  //
  // Zero = sem limite, que e o normal para programa comum.
  m.limiteManualDeTaxaBps = (int)c.taxaManualBps;

  // Os tetos de pedaco, quando o aplicativo mandou algum. Zero = padrao daqui.
  if (c.chunkInicial > 0)     m.pedacoInicialBytes  = (int)c.chunkInicial;
  if (c.chunkMin > 0)         m.pedacoMinimoBytes   = (int)c.chunkMin;
  if (c.chunkMax > 0)         m.pedacoMaximoBytes   = (int)c.chunkMax;
  if (c.holdSemConsumoMs > 0) m.esperaSemConsumoMs  = (int)c.holdSemConsumoMs;
  if (c.debounceMs > 0)       m.debounceDeRetomadaMs = (int)c.debounceMs;

  m.limiteDoBufferDeEscrita =
      (c.flow == FLOW_XONXOFF || c.flow == FLOW_RTSCTS) ? JANELA_XONXOFF
                                                         : JANELA_DE_ESCRITA;
  return m;
}

// --- Protocolo com o aplicativo Android -------------------------------------

// Traduz a configuracao que veio pelo protocolo para a do firmware.
//
// OS NUMEROS NAO COINCIDEM, E ISSO E UMA ARMADILHA.
//
// O aplicativo manda paridade nos valores do Qt (0 nenhuma, 2 par, 3 impar) e
// fluxo no vocabulario da tela (0 nenhum, 1 XON/XOFF, 3 RTS/CTS). No firmware,
// paridade e 0/1/2 e fluxo e 0/1/2/3, onde 3 significa DTR/DSR.
//
// Um `cast` direto entregaria DTR/DSR quando o operador escolheu RTS/CTS -- ou
// seja, controle de fluxo errado, que e a receita do transbordo. Por isso a
// traducao e caso a caso e explicita.
static DncSerialConfig traduzirConfigDoProtocolo(const smi::dnc::esp::ConfigSerial& c) {
  DncSerialConfig d;
  d.baudRate = c.baud;
  d.dataBits = c.dataBits;

  switch (c.paridade) {
    case 2:  d.parity = PARITY_EVEN; break;   // Qt EvenParity
    case 3:  d.parity = PARITY_ODD;  break;   // Qt OddParity
    default: d.parity = PARITY_NONE; break;
  }

  d.stopBits = (c.stopBits >= 2) ? 2 : 1;

  // "PELO APP" E "NATIVO DO CHIP" SAO A MESMA COISA AQUI. E TEM QUE SER.
  //
  // A tela do aplicativo oferece cinco opcoes, herdadas do caminho por CABO:
  //   0 nenhum
  //   1 XON/XOFF pelo app     2 XON/XOFF nativo do chip
  //   3 RTS/CTS  pelo app     4 RTS/CTS  nativo do chip
  //
  // Essa distincao so faz sentido no cabo, onde da para escolher entre o
  // aplicativo interpretar o freio ou o conversor USB fazer isso sozinho.
  // QUANDO QUEM FALA COM A MAQUINA E ESTA CAIXA, quem freia e SEMPRE a caixa --
  // o aplicativo nem esta no circuito, ele so entrega o arquivo e manda tocar.
  //
  // O QUE ISSO CAUSAVA: 2 e 4 caiam no `default` e viravam FLOW_NONE. Escolher
  // "nativo do chip" na tela DESLIGAVA o controle de fluxo da caixa inteira --
  // sem ler XOFF, sem olhar CTS, despejando o arquivo do inicio ao fim. Nao
  // havia aviso: a tela dizia "XON/XOFF" e o fio nao tinha freio nenhum.
  // Programa curto passa porque a maquina nunca precisa frear; programa grande
  // alarma na primeira vez que ela tenta.
  //
  // Um valor que o aplicativo oferece NUNCA pode significar "sem protecao" por
  // acidente de tradução. Os dois sinonimos levam ao mesmo freio.
  switch (c.fluxo) {
    case 1:
    case 2:  d.flow = FLOW_XONXOFF; break;
    case 3:                                   // 3 no app = RTS/CTS, NAO DTR/DSR
    case 4:  d.flow = FLOW_RTSCTS;  break;
    default: d.flow = FLOW_NONE;    break;
  }

  // OS AJUSTES DO MOTOR TAMBEM VEM PELO PROTOCOLO. NAO DEIXE CAIR.
  d.taxaManualBps    = c.taxaManualBps;
  d.chunkInicial     = c.chunkInicial;
  d.chunkMin         = c.chunkMin;
  d.chunkMax         = c.chunkMax;
  d.holdSemConsumoMs = c.holdSemConsumoMs;
  d.debounceMs       = c.debounceMs;

  d.timeoutMs = copiarConfigSerial().timeoutMs;  // nao vem pelo protocolo
  return d;
}

// Traduz o estado INTERNO do firmware para o estado do PROTOCOLO.
//
// SAO DOIS ENUMS DIFERENTES, E MANDAR UM NO LUGAR DO OUTRO E DESASTRE.
//
// Foi o que eu fiz na primeira ligacao, e o teste de integracao pegou: o
// firmware publicava o proprio DncState, e o aplicativo o lia como o Estado do
// protocolo. O resultado, na tela do operador:
//
//   ERRO (8)                   virava  COMPLETED   -- falha aparecia como sucesso
//   Aguardando confirmacao (9) virava  CANCELLED
//   Drenagem travada (10)      virava  ERROR
//
// Um erro na maquina se apresentando como "concluido" e a pior confusao
// possivel num DNC. Por isso a traducao e explicita, caso a caso, e nunca um
// cast.
// Devolvido quando o firmware NAO tem nada a dizer sobre o estado.
static const uint8_t kNaoPublicar = 0xFF;

static uint8_t estadoDoProtocolo(DncState s) {
  namespace E = smi::dnc::esp;
  switch (s) {
    // NADA A PUBLICAR ENQUANTO A MAQUINA ESTA PARADA.
    //
    // Sao DOIS EIXOS de estado, e trata-los como um so quebra o upload:
    //   - as fases do ARQUIVO (recebendo, validando, pronto) pertencem ao
    //     nucleo do protocolo, que e quem as conhece;
    //   - as fases da OPERACAO com a maquina pertencem ao firmware.
    //
    // Publicar IDLE/READY daqui sobrescrevia a maquina de estados do nucleo no
    // meio da subida do arquivo -- o bloco seguinte era recusado com
    // "FILE_BLOCK recusado no estado READY". Enquanto nao ha operacao em curso,
    // o firmware nao tem nada a dizer: quem manda no estado e o nucleo.
    case DNC_IDLE:
    case DNC_CONNECTED:     return kNaoPublicar;

    case DNC_SENDING:       return E::EST_SENDING;
    case DNC_DRIP_FEED:     return E::EST_ONLINE;
    case DNC_RECEIVING:     return E::EST_RECEIVING_FROM_CNC;
    case DNC_PAUSED_USER:   return E::EST_PAUSED;

    // Pausa por controle de fluxo NAO e pausa do operador: a maquina mandou
    // esperar e a transmissao continua em andamento. Mostrar "pausado" faria o
    // operador procurar quem pausou.
    case DNC_PAUSED_FLOW:   return E::EST_SENDING;

    case DNC_CANCELING:     return E::EST_CANCELLED;
    case DNC_ERROR:         return E::EST_ERROR;
    case DNC_AWAIT_FINISH:  return E::EST_AWAIT_FINISH;
    case DNC_DRAIN_STALLED: return E::EST_DRAIN_STALLED;

    // Fim da recepcao. E daqui que sai o aviso que faz o aplicativo ir buscar
    // o arquivo sozinho -- ver a nota em DNC_RECEIVE_DONE.
    case DNC_RECEIVE_DONE:  return E::EST_COMPLETED;
    case DNC_RECEIVE_IDLE:  return E::EST_RECEIVE_IDLE;
    default:                return kNaoPublicar;
  }
}

// Liga o servidor de protocolo. Chamado depois da rede estar de pe.
static void iniciarProtocolo() {
  auto& acoes = gProtocolo.acoes();

  acoes.aoAplicarConfig = [](const smi::dnc::esp::ConfigSerial& c) -> bool {
    const DncSerialConfig nova = traduzirConfigDoProtocolo(c);
    String erro;
    if (!validarConfigSerial(nova, erro)) {
      logEventf("ERRO", "Configuracao serial do aplicativo recusada: %s", erro.c_str());
      return false;   // vira NACK: o aplicativo precisa SABER que nao pegou
    }
    atualizarConfigSerial(nova);
    settings.saveSerial(nova);
    DncCommand cmd;
    cmd.type = CMD_APPLY_SERIAL;
    enfileirarComandoDnc(cmd, 50);
    logEventf("INFO", "Config recebida do aplicativo: %lu %u%s%u fluxo=%u",
              (unsigned long)nova.baudRate, nova.dataBits,
              parityToText(nova.parity), nova.stopBits, (unsigned)nova.flow);
    return true;
  };

  // --- Rede sem fio: instalar caixa nova sem tecnico ------------------------

  acoes.aoPerguntarSeTemRede = []() -> bool {
    // "Primeira instalacao" e: nao ha rede gravada, OU ha mas a caixa nao
    // conseguiu entrar e esta servindo pelo ponto de acesso de socorro. Os dois
    // casos levam a mesma tela no aplicativo.
    const NetworkConfig rede = copiarConfigRede();
    if (rede.ssid.length() == 0) return true;
    return WiFi.status() != WL_CONNECTED;
  };

  acoes.aoVarrerRedes = []() -> std::vector<smi::dnc::esp::IAcoesDnc::RedeVista> {
    std::vector<smi::dnc::esp::IAcoesDnc::RedeVista> fora;

    // O RADIO PRECISA ESTAR LIVRE PARA VARRER.
    //
    // MEDIDO EM CAMPO: a caixa "nao enxergou nenhuma rede" na instalacao, e o
    // log mostrava -2 -- que NAO e "zero redes", e WIFI_SCAN_FAILED.
    //
    // A causa: numa caixa sem rede valida a TaskNetwork fica tentando conectar
    // em laco, e `scanNetworks` falha quando a estacao esta no meio de uma
    // tentativa. Ou seja, justamente na instalacao -- o unico momento em que
    // esta varredura importa -- ela nunca ia funcionar.
    //
    // Soltar a estacao antes de varrer resolve. So quando ela NAO esta
    // conectada: numa caixa em producao, derrubar a conexao para varrer seria
    // trocar um problema por outro bem pior.
    esp_task_wdt_reset();

    // Resultado velho entope a proxima varredura. Limpar sempre, nao so no
    // caminho feliz -- era outro defeito deste mesmo trecho.
    WiFi.scanDelete();

    const bool conectada = (WiFi.status() == WL_CONNECTED);
    if (!conectada) {
      WiFi.disconnect(false, false);
      delay(150);
    }

    int achados = WiFi.scanNetworks(false, true);
    esp_task_wdt_reset();

    // Uma falha isolada acontece (o radio pode estar ocupado com o proprio
    // ponto de acesso). Uma segunda tentativa custa pouco e evita mandar o
    // instalador embora achando que a caixa esta cega.
    if (achados == WIFI_SCAN_FAILED) {
      logEvent("INFO", "Varredura falhou; tentando mais uma vez.");
      WiFi.scanDelete();
      delay(400);
      esp_task_wdt_reset();
      achados = WiFi.scanNetworks(false, true);
      esp_task_wdt_reset();
    }

    if (achados <= 0) {
      logEventf("WARN", "Varredura a pedido do aplicativo: %s",
                achados == WIFI_SCAN_FAILED ? "FALHOU (radio ocupado)"
                                            : "nenhuma rede no ar");
      WiFi.scanDelete();
      return fora;
    }

    for (int i = 0; i < achados; ++i) {
      const String ssid = WiFi.SSID(i);
      if (ssid.length() == 0) continue;            // rede oculta, sem nome util

      // Sem repetir: a mesma rede aparece uma vez por ponto de acesso, e o
      // operador so precisa do nome. Fica o de sinal mais forte.
      bool repetida = false;
      for (auto& j : fora) if (j.ssid == std::string(ssid.c_str())) { repetida = true; break; }
      if (repetida) continue;

      smi::dnc::esp::IAcoesDnc::RedeVista r;
      r.ssid      = std::string(ssid.c_str());
      r.sinal     = (int8_t)WiFi.RSSI(i);
      r.protegida = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
      fora.push_back(r);
    }
    WiFi.scanDelete();

    // Mais forte primeiro: e a ordem em que o operador quer escolher.
    std::sort(fora.begin(), fora.end(),
              [](const smi::dnc::esp::IAcoesDnc::RedeVista& a,
                 const smi::dnc::esp::IAcoesDnc::RedeVista& b) { return a.sinal > b.sinal; });

    logEventf("INFO", "Varredura a pedido do aplicativo: %u redes uteis",
              (unsigned)fora.size());
    return fora;
  };

  acoes.aoGravarRede = [](const std::string& ssid, const std::string& senha) -> bool {
    NetworkConfig rede = copiarConfigRede();
    rede.ssid     = String(ssid.c_str());
    rede.password = String(senha.c_str());
    atualizarConfigRede(rede);
    settings.saveNetwork(rede);

    // A troca em si fica para a TaskNetwork: ver a nota em gTrocaDeRedePedida.
    gRedeNovaSsid  = rede.ssid;
    gRedeNovaSenha = rede.password;
    gTrocaDeRedePedida = true;

    logEventf("OK", "Rede '%s' cadastrada pelo aplicativo. Entrando nela.",
              rede.ssid.c_str());
    return true;
  };

  // SO ENFILEIRA. Quem executa continua sendo a TaskDNC, exatamente como quando
  // o comando vem do portal web -- e por isso a UART da maquina nao e tocada
  // por esta camada.
  acoes.aoIniciar = [](uint8_t modo, const String& nomeArquivo) -> bool {
    DncCommand cmd;
    cmd.type = (modo == 1) ? CMD_DRIP_FILE : (modo == 2 ? CMD_RECEIVE_FILE : CMD_SEND_FILE);

    // A PASTA DEPENDE DO SENTIDO.
    //
    // Enviar LE de /programs; receber GRAVA em /received. Jogar o recebido em
    // /programs misturaria o que veio da maquina com o que o operador preparou
    // -- e um nome repetido sobrescreveria um programa bom.
    const bool recepcao = (modo == 2);
    const String pasta = recepcao ? String(DIR_RECEIVED) : String(DIR_PROGRAMS);

    String caminho;
    if (recepcao && nomeArquivo.length() == 0) {
      // Sem nome pedido, quem nomeia e o firmware (pelo tempo de sessao).
      // Caminho vazio e o sinal disso para o receiveFromCnc.
      caminho = "";
    } else {
      caminho = storage.sanitizePath(pasta + "/" + storage.sanitizeFileName(nomeArquivo));
    }

    strncpy(cmd.path, caminho.c_str(), sizeof(cmd.path) - 1);
    return enfileirarComandoDnc(cmd, 50);
  };

  acoes.aoPausar = []() {
    solicitarPausaDnc("aplicativo");
  };

  acoes.aoRetomar = []() {
    solicitarRetomadaDnc("aplicativo");
  };

  acoes.aoCancelar = []() {
    solicitarCancelamentoDnc("aplicativo");
  };

  acoes.aoFinalizar = []() {
    // Na recepcao, "finalizar" quer dizer ENCERRAR E GUARDAR o que ja veio.
    if (dncEstadoAtual() == DNC_RECEIVE_IDLE) { gRecepcaoFinalizar = true; return; }
    finalizarSessaoPeloOperador();
    logEvent("INFO", "Encerramento confirmado pelo aplicativo");
  };

  acoes.aoContinuarAguardando = []() {
    // Na recepcao, "continuar aguardando" reabre a janela de silencio.
    if (dncEstadoAtual() == DNC_RECEIVE_IDLE) { gRecepcaoContinuar = true; return; }
    continuarAguardandoDrenagem();
    logEvent("INFO", "Aplicativo optou por continuar aguardando a maquina");
  };

  // Quais linhas RS232 esta placa sabe LER: bit0 = CTS, bit1 = DSR.
  //
  // As DUAS existem nas duas placas, ambas ligadas a receivers de MAX3232
  // (pinos em Config.h). Eu tinha deixado `1` aqui, e o teste de integracao mostrou o
  // aplicativo recebendo "DSR=0" -- ou seja, a tela diria ao operador que a
  // placa nao le DSR quando ela le.
  //
  // Isso importa porque o LED apagado e ambiguo: sem esta informacao, o
  // operador nao distingue "linha em nivel baixo" de "este equipamento nao le
  // essa linha". Declarar menos do que existe e tao ruim quanto declarar mais.
  const uint8_t kLinhaCts = 0x01;
  const uint8_t kLinhaDsr = 0x02;
  const NetworkConfig rede = copiarConfigRede();
  gProtocolo.begin(DNC_PROTOCOL_PORT, storage.mutex(), DIR_PROGRAMS,
                   FIRMWARE_VERSION, rede.nomeDoEquipamento,
                   kLinhaCts | kLinhaDsr);
  logEventf("INFO", "Protocolo do aplicativo escutando na porta %u", (unsigned)DNC_PROTOCOL_PORT);

  // Descoberta. Responde "quem sou e onde estou" -- nada mais. Comando continua
  // so pela porta do protocolo, com controle exclusivo.
  gDescoberta.definirVerificadorDeOcupado(dncOcupado);
  gDescoberta.definirEspacoLivre([]() -> uint64_t { return storage.freeBytes(); });
  gDescoberta.begin(rede.nomeDoEquipamento, FIRMWARE_VERSION, DNC_PROTOCOL_PORT);
  logEventf("INFO", "Descoberta na rede ativa na porta %u",
            (unsigned)smi::dnc::esp::kPortaDescoberta);
}

// --- Respostas do operador a sessao de envio --------------------------------
//
// Chamadas do portal web (e, mais adiante, do aplicativo pelo protocolo). Ficam
// aqui porque e aqui que o motor e a sua tranca moram.

void finalizarSessaoPeloOperador() {
  travarMotor();
  gSessao.finalizar(millis());
  destravarMotor();
}

void continuarAguardandoDrenagem() {
  travarMotor();
  gSessao.continuarAguardando(millis());
  destravarMotor();
}

// Qual das duas particoes de aplicativo esta rodando: "app0" ou "app1".
//
// ESTA E A PROVA DE QUE UMA ATUALIZACAO PELO AR PEGOU. O OTA nunca grava por
// cima do firmware em uso: ele escreve na particao livre e so aponta o boot
// para la depois de conferir o arquivo inteiro. Entao a particao TROCA a cada
// atualizacao bem-sucedida -- e continua a mesma quando a atualizacao falhou,
// mesmo que o envio pareca ter terminado.
//
// Sem isto, a unica forma de saber se a caixa de um cliente recebeu a versao
// nova seria ir ate la.
String particaoEmUso() {
  const esp_partition_t* p = esp_ota_get_running_partition();
  return p && p->label ? String(p->label) : String("desconhecida");
}

// True enquanto houver qualquer operacao aberta com a maquina.
//
// PAUSADO TAMBEM CONTA COMO OCUPADO. Um envio pausado nao acabou: o programa
// esta pela metade na CNC e o operador vai retomar. O OTA reinicia o ESP32 ao
// terminar, e reiniciar nesse momento perde a sessao do mesmo jeito que
// reiniciar no meio da transmissao.
static DncState dncEstadoAtual() {
  DncState s;
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  s = gStatus.state;
  xSemaphoreGive(gStatusMutex);
  return s;
}

static bool dncEnvioAtivo() {
  bool ativo;
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  ativo = gStatus.state == DNC_SENDING || gStatus.state == DNC_DRIP_FEED
       || gStatus.state == DNC_PAUSED_FLOW
       || (gStatus.state == DNC_PAUSED_USER
           && (gStatus.resumeState == DNC_SENDING
               || gStatus.resumeState == DNC_DRIP_FEED));
  xSemaphoreGive(gStatusMutex);
  return ativo;
}

static bool dncOcupado() {
  DncState s;
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  s = gStatus.state;
  xSemaphoreGive(gStatusMutex);
  return s == DNC_SENDING || s == DNC_RECEIVING || s == DNC_DRIP_FEED
      || s == DNC_PAUSED_USER || s == DNC_PAUSED_FLOW || s == DNC_CANCELING
      || s == DNC_AWAIT_FINISH || s == DNC_DRAIN_STALLED
      || s == DNC_RECEIVE_IDLE;
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

  // O CORE JA INICIALIZOU O WATCHDOG ANTES DE NOS.
  //
  // No core 3.x o watchdog de tarefa ja vem de pe, e este esp_task_wdt_init()
  // falhava com "TWDT already initialized" -- em silencio, do ponto de vista do
  // firmware. O resultado e que os 15000 ms aqui embaixo NUNCA valeram: valia o
  // padrao do core, bem menor. O codigo operava achando que tinha uma folga que
  // nao tinha, e isso derruba o equipamento em qualquer operacao demorada.
  //
  // Reconfigurar e o caminho certo quando ja esta inicializado.
  esp_err_t r = esp_task_wdt_init(&twdt_config);
  if (r == ESP_ERR_INVALID_STATE) {
    r = esp_task_wdt_reconfigure(&twdt_config);
  }
  if (r != ESP_OK) {
    Serial.printf("ERRO: watchdog nao configurado (%d). Timeout e o do core.\n", (int)r);
  }
#else
  esp_task_wdt_init(15, true);
#endif
}

// ---------- Rede ----------

// POR QUE O MOTIVO DA QUEDA PRECISA APARECER NO LOG.
//
// A caixa sumia da rede por ~40 s e voltava sozinha, sem reiniciar e sem
// imprimir nada -- porque quem reconecta e a propria pilha do Wi-Fi, por baixo
// do nosso codigo. Sem o motivo, "perde conexao toda hora" nao tem como virar
// diagnostico: sinal fraco, roteador expulsando o cliente e senha recusada
// produzem exatamente o mesmo sintoma visto de fora.
//
// O numero do motivo separa os tres:
//   200 BEACON_TIMEOUT / 8 ASSOC_LEAVE ..... sinal ou roteador soltando
//   201 NO_AP_FOUND ....................... nao enxergou a rede
//   2 AUTH_EXPIRE / 15 HANDSHAKE_TIMEOUT .. autenticacao
//   205 CONNECTION_FAIL .................... falha ao associar
static void aoEventoDeWiFi(WiFiEvent_t evento, WiFiEventInfo_t info) {
  switch (evento) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      logEventf("INFO", "Wi-Fi associado (canal %u, sinal %d dBm)",
                (unsigned)WiFi.channel(), (int)WiFi.RSSI());
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      logEventf("OK", "Wi-Fi com IP %s (sinal %d dBm)",
                WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      logEventf("WARN", "Wi-Fi CAIU (motivo %u, ultimo sinal %d dBm)",
                (unsigned)info.wifi_sta_disconnected.reason, (int)WiFi.RSSI());
      break;

    default:
      break;
  }
}

static void startNetwork() {
  const NetworkConfig rede = copiarConfigRede();
  WiFi.onEvent(aoEventoDeWiFi);
  // NOME PROPRIO DESTA CAIXA, nao a constante do codigo.
  //
  // E este nome que aparece na lista de aparelhos do roteador e no OTA. Com a
  // constante, cinco caixas na mesma fabrica apareciam identicas -- e gravar
  // firmware na maquina errada era questao de tempo.
  WiFi.setHostname(rede.nomeDoEquipamento.c_str());
  WiFi.mode(WIFI_STA);

  // DESLIGAR A ECONOMIA DE ENERGIA DO RADIO. ESTA CAIXA VIVE NA TOMADA.
  //
  // O core do ESP32 liga modem sleep por padrao: o radio dorme entre os
  // beacons do roteador e so acorda de tempos em tempos. Num aparelho a pilha
  // isso faz sentido; aqui nao economiza nada e custa caro.
  //
  // MEDIDO EM BANCADA, com a caixa e o PC na mesma rede: a resposta do portal
  // oscilava entre 20 ms e 337 ms, agrupada em dois patamares -- o de ~130 ms
  // e exatamente o intervalo de beacon, ou seja, o pedido esperando o radio
  // acordar. Esse tremor e o que derruba sessao quando ha trafego de verdade:
  // o aplicativo tem prazo de resposta, e um pedido que as vezes demora dez
  // vezes mais estoura esse prazo sem que nada esteja quebrado.
  WiFi.setSleep(false);

  // CONECTAR NO PONTO DE ACESSO MAIS FORTE, NAO NO PRIMEIRO QUE APARECER.
  //
  // Por padrao o ESP32 para a varredura no primeiro ponto que responde com o
  // SSID procurado. Numa fabrica com varios pontos anunciando a MESMA rede,
  // isso significa grudar num distante e ficar no limite do alcance -- com
  // quedas constantes -- tendo outro bem mais perto disponivel.
  //
  // Medido nesta caixa: -46 dBm numa rede e -74 dBm na outra. Vinte e cinco
  // decibeis de diferenca e, grosso modo, trezentas vezes menos potencia, e e
  // nessa faixa que a transferencia comeca a ser cortada no meio.
  //
  // Varrer todos os canais custa alguns segundos a mais para conectar. Vale:
  // um programa cortado no meio custa a peca.
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);

  // Potencia de transmissao no maximo, explicitamente.
  //
  // O padrao varia com a versao do core, e um equipamento de painel nao tem
  // motivo nenhum para transmitir fraco: nao e alimentado a bateria.
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  // Reconectar e trabalho da pilha, nao so da TaskNetwork.
  WiFi.setAutoReconnect(true);

  // Nao regravar as credenciais na flash a cada WiFi.begin().
  //
  // Quem guarda SSID e senha aqui e o nosso Settings, na NVS. Deixar a pilha
  // gravar as dela por cima a cada tentativa so gasta a flash -- e a tentativa
  // de reconexao roda em laco quando a rede cai, que e justamente quando isso
  // mais aconteceria.
  WiFi.persistent(false);

  // PARAR E APAGAR O QUE A PILHA GUARDOU, ANTES DE PEDIR A REDE NOVA.
  //
  // A pilha do Wi-Fi guarda por conta as credenciais da ultima conexao que deu
  // certo, e com auto-reconexao ligada ela JA COMECA a conectar com elas antes
  // do nosso codigo falar. Quando o nosso WiFi.begin() chega com o SSID novo,
  // o driver responde "sta is connecting, cannot set config" e IGNORA o pedido.
  //
  // O sintoma em bancada foi exato: trocar o SSID na tela, salvar, reiniciar --
  // e a caixa continuava entrando na rede ANTIGA, como se a troca nao existisse.
  //
  // disconnect(false, true): nao desliga o radio, mas APAGA a configuracao
  // guardada. Depois disso o unico SSID que existe e o nosso.
  WiFi.disconnect(false, true);
  delay(100);

  if (rede.ssid.length()) {
    if (!rede.dhcp) {
      WiFi.config(rede.ip, rede.gateway, rede.subnet, rede.dns);
    } else {
      // VOLTAR PARA DHCP EXIGE CANCELAR O IP FIXO EXPLICITAMENTE.
      //
      // Nao chamar WiFi.config() NAO restaura o DHCP: um endereco fixo aplicado
      // antes continua valendo na interface. Na pratica, marcar DHCP na tela e
      // salvar parecia nao fazer efeito -- a caixa seguia no IP antigo, e o
      // operador ficava sem entender por que a mudanca nao pegou.
      //
      // Endereco nulo nos quatro campos e a forma documentada de dizer
      // "peca por DHCP".
      WiFi.config((uint32_t)0, (uint32_t)0, (uint32_t)0, (uint32_t)0);
    }
    WiFi.begin(rede.ssid.c_str(), rede.password.c_str());
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
      // ALIMENTAR O WATCHDOG AQUI E OBRIGATORIO.
      //
      // delay() no Arduino ESP32 devolve a CPU ao sistema, mas NAO alimenta o
      // watchdog de tarefa. Sao ate 8 segundos parados aqui, e esta funcao roda
      // dentro do setup(), numa tarefa que ja esta sendo vigiada. Sem este
      // reset, um Wi-Fi que demora a responder derruba o equipamento no boot --
      // e como este trecho so roda quando ha um SSID salvo, o defeito so
      // aparece DEPOIS que o cliente configura a rede.
      esp_task_wdt_reset();
      delay(250);
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    // AP_STA, NAO AP PURO. O REFUGIO NAO PODE SER UM BECO SEM SAIDA.
    //
    // Com WIFI_AP a caixa parava de tentar a rede da fabrica PARA SEMPRE: a
    // TaskNetwork so reconecta quando o modo e de estacao, e o modo tinha
    // acabado de deixar de ser. Bastava o roteador demorar mais que os 8
    // segundos deste boot -- reinicio de energia, roteador subindo junto com a
    // fabrica de manha -- e o equipamento sumia da rede ate alguem tirar da
    // tomada. E ninguem no cliente vai adivinhar que precisa procurar uma rede
    // chamada SMI_DNC_ESP32 para trazer a maquina de volta.
    //
    // Em AP_STA o ponto de acesso de socorro existe E a tentativa continua. A
    // caixa volta sozinha quando o Wi-Fi voltar.
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);   // vale tambem no socorro: e por aqui que se configura a rede
    WiFi.softAP(DEFAULT_AP_SSID, DEFAULT_AP_PASS);
    if (rede.ssid.length()) {
      WiFi.begin(rede.ssid.c_str(), rede.password.c_str());
    }
    // QUANTOS PONTOS DE ACESSO EXISTEM COM ESSE NOME, E COM QUE SINAL.
    //
    // Responde de uma vez a pergunta que separa "problema de firmware" de
    // "problema de infraestrutura": se aparecerem varios pontos com a mesma
    // rede e sinais muito diferentes, a caixa esta no lugar errado da fabrica
    // e nenhuma linha de codigo conserta isso.
    // Mesma armadilha de aoVarrerRedes: varrer logo depois do begin() pega o
    // radio ocupado e devolve -2. Aqui e so diagnostico, entao nao vale
    // derrubar a tentativa de conexao -- mas vale DIZER que falhou, em vez de
    // registrar "-2 redes no ar", que nao quer dizer nada.
    const int achados = WiFi.scanNetworks();
    if (achados < 0) {
      logEventf("INFO", "Varredura de diagnostico nao pode ser feita agora (%d).",
                achados);
    } else {
      logEventf("INFO", "Varredura: %d redes no ar", achados);
    }
    for (int i = 0; i < achados && i < 12; ++i) {
      const bool minha = (WiFi.SSID(i) == rede.ssid);
      logEventf("INFO", "  %s%s  canal %d  sinal %d dBm",
                minha ? "[A MINHA] " : "", WiFi.SSID(i).c_str(),
                WiFi.channel(i), WiFi.RSSI(i));
    }
    WiFi.scanDelete();

    logEventf("INFO", "AP de socorro: %s IP %s (continua tentando entrar em '%s')",
              DEFAULT_AP_SSID, WiFi.softAPIP().toString().c_str(),
              rede.ssid.c_str());
  } else {
    logEventf("INFO", "Wi-Fi conectado: %s IP %s", rede.ssid.c_str(), WiFi.localIP().toString().c_str());
  }
}

// ---------- Atualizacao de firmware pelo Wi-Fi ----------
//
// Chamado no setup e, se falhar por Wi-Fi ainda nao pronto, de novo pela
// TaskNetwork quando a conexao subir. Nao ha nada a fazer se a senha estiver
// vazia: o proprio OtaUpdater recusa, e essa recusa e a configuracao de fabrica.
static void iniciarOta() {
  if (gOta.iniciado()) return;
  const NetworkConfig rede = copiarConfigRede();

  // SEM SENHA NAO INSISTE.
  //
  // Esta funcao roda a cada volta da TaskNetwork, uma vez por segundo. Chamar
  // begin() sem senha registraria a recusa uma vez por segundo para sempre,
  // enchendo a fila de log e empurrando as mensagens uteis para fora do anel de
  // logs recentes. Avisa uma vez e volta a tentar so quando houver senha -- o
  // portal web escreve em gNetworkConfig na hora, sem exigir reinicio.
  if (rede.otaPassword.length() == 0) {
    static bool avisado = false;
    if (!avisado) {
      avisado = true;
      logEvent("INFO", "OTA desligado: defina a senha de atualizacao na aba Rede.");
    }
    return;
  }

  gOta.definirVerificadorDeOcupado(dncOcupado);
  gOta.definirLogger([](const String& t) { logEvent("INFO", t.c_str()); });
  gOta.begin(rede.nomeDoEquipamento, rede.otaPassword);
}

// ---------- TaskSerial: UART, RX/TX, fluxo ----------
void TaskSerial(void* pv) {
  const uint8_t meuIndice = TAREFA_SERIAL;
  strncpy(gNomeDaTarefa[TAREFA_SERIAL], "SERIAL", 11);
  gNomeDaTarefa[TAREFA_SERIAL][11] = 0;
  setupWatchdogForCurrentTask();
  logEvent("INFO", "TaskSerial iniciada");

  // O TETO DE 1000 B/s ERA AQUI.
  //
  // A versao anterior escrevia UM byte por volta e dormia 1 ms no fim -- teto
  // fixo de 1000 bytes por segundo, independente da configuracao. A 9600 baud
  // isso passava despercebido por sorte (o fio entrega ~960 B/s), mas a 19200
  // ou mais o ESP32 alimentava a maquina na metade da velocidade que ela
  // aceita, e a usinagem esperava por nada.
  //
  // Agora cada volta move um PEDACO dimensionado pelo motor. As tres regras
  // que impedem o transbordo continuam valendo, e sao elas que tornam o
  // aumento seguro:
  //   1. o agendador ja limita o pedaco pelo que a maquina consumiu;
  //   2. `writeBlock` nunca escreve mais do que cabe no buffer de saida;
  //   3. o dado so sai da fila DEPOIS de a UART confirmar (espiar/descartar).

  uint8_t blocoRx[256];
  uint8_t blocoTx[256];

  uint32_t ultimoLogDeFluxo = 0;
  uint8_t  ultimoMotivoLogado = 0xFF;   // 0xFF = nada registrado ainda
  uint32_t ultimoStatusMs   = 0;
  bool permitidoAntes = true;
  uint8_t fluxoAplicado = 0xFF;   // forca a primeira configuracao do motor

  for (;;) {
    esp_task_wdt_reset(); baterPonto(meuIndice);
    const uint32_t agora = millis();
    const DncSerialConfig configSerial = copiarConfigSerial();

    // A configuracao da maquina pode mudar em tempo de execucao (portal ou
    // aplicativo). Reconfigura o motor so quando mudou de verdade: reconfigurar
    // a cada volta zeraria o aprendizado do agendador mil vezes por segundo.
    if (fluxoAplicado != (uint8_t)configSerial.flow) {
      fluxoAplicado = (uint8_t)configSerial.flow;
      gConfigMotor = traduzirConfigDoMotor(configSerial);
      gAgendador.definirConfig(gConfigMotor);
      gFluxo.definirConfig(gConfigMotor);
      gVigia.definirConfig(gConfigMotor);
    }

    // --- Leitura da CNC, em bloco -------------------------------------------
    size_t lidos = serialManager.readBlock(blocoRx, sizeof(blocoRx));
    if (lidos > 0) {
      const DncState estadoAgora = dncEstadoAtual();
      const bool enviando = dncEnvioAtivo();
      uint32_t dados = 0;
      uint32_t descartados = 0;
      for (size_t i = 0; i < lidos; ++i) {
        const uint8_t b = blocoRx[i];
        // XOFF VALE MESMO EM RTS/CTS, ENQUANTO ESTAMOS ENVIANDO.
        //
        // Muitos controles -- Siemens entre eles -- deixam o RTS acionado o
        // tempo todo, como "linha pronta", e fazem o freio de verdade com
        // DC1/DC3. Configurado em RTS/CTS, o firmware ignorava o XOFF: a
        // maquina mandava "pare", nos tratavamos como dado e continuavamos
        // despejando. A Siemens acusou "overrun buffer" carregando NCK, a 9600
        // E a 57600 -- transbordo que nao muda com a velocidade e sinal de freio
        // ignorado, nao de velocidade demais.
        //
        // SO ENQUANTO ENVIAMOS. Durante a recepcao, 0x11 e 0x13 podem ser dados
        // legitimos do arquivo, e engoli-los corromperia o programa recebido.
        const bool leXonXoff = enviando
                            && (configSerial.flow == FLOW_XONXOFF
                                || configSerial.flow == FLOW_RTSCTS);

        if (leXonXoff && b == 0x13) {                                   // XOFF
          gXonAllowed = false;
          gFluxo.aoReceberXoff();
          travarMotor(); gSessao.aoReceberXoff(); destravarMotor();
          setState(DNC_PAUSED_FLOW);

          // MEDIR O QUE A FANUC CONTA PARA ALARMAR 087.
          //
          // Ela aceita NO MAXIMO 10 CARACTERES depois do DC3. O numero que
          // decide isso e quanto ainda esta na fila de saida NESTE INSTANTE:
          //   - se o freio for do periferico, este monte NAO sai -- fica preso;
          //   - se for do nosso laco, ele sai inteiro no fio e a maquina alarma.
          //
          // Por isso medimos duas vezes: agora e um pouco depois. A DIFERENCA e
          // a resposta, e ela dispensa qualquer teoria sobre o silicio.
          // Com o freio no periferico este caractere e consumido pelo
          // hardware e nao chega aqui. Se chegar, e porque o freio ficou no
          // software -- e a linha abaixo diz exatamente isso, com o tamanho da
          // fila que vai escapar no fio.
          logEventf("WARN", "XOFF chegou ao software (freio NAO esta no "
                            "hardware) - %u B na fila vao escapar",
                    (unsigned)serialManager.bytesNaSaida());
        } else if (leXonXoff && b == 0x11) {                            // XON
          gXonAllowed = true;
          gFluxo.aoReceberXon();
          travarMotor(); gSessao.aoReceberXon(); destravarMotor();
          retomarEstadoAposFluxo();
          logEvent("FLOW", "XON recebido - transmissao retomada pela CNC");
        } else {
          if (rxRing.push(b)) ++dados;
          else ++descartados;
        }
      }
      if (descartados > 0) {
        gRxOverflowCount += descartados;
        xSemaphoreTake(gStatusMutex, portMAX_DELAY);
        gStatus.rxOverflowCount += descartados;
        xSemaphoreGive(gStatusMutex);
        if (estadoAgora == DNC_RECEIVING && configSerial.flow == FLOW_XONXOFF) {
          serialManager.enviarControleDeFluxo(0x13);
        }
        logEventf("ERRO", "Buffer RX cheio: %lu byte(s) perdido(s)",
                  (unsigned long)descartados);
      }
      if (dados > 0) {
        // Um bloqueio por bloco, nao um por byte: a versao anterior pegava o
        // mutex de estado a cada byte recebido.
        xSemaphoreTake(gStatusMutex, portMAX_DELAY);
        gStatus.bytesReceived += dados;
        gStatus.lastByteMs = agora;
        xSemaphoreGive(gStatusMutex);
      }
    }

    // O FREIO AGORA E VISTO PELA FILA, NAO PELO CARACTERE.
    //
    // Com o freio no periferico o XOFF e consumido pelo proprio hardware e NAO
    // chega mais ao nosso codigo -- entao "XOFF recebido" deixou de existir. Mas
    // o efeito continua observavel: a fila de saida PARA DE DRENAR enquanto a
    // maquina mantem o pare.
    //
    // Isto vale mais que o caractere: mede o que de fato acontece no fio, e
    // funciona igual para o freio por CTS, que nunca teve caractere nenhum.
    {
      static uint32_t filaAnterior   = 0;
      static uint32_t paradaDesdeMs  = 0;
      static bool     avisouParado   = false;

      const uint32_t fila = (uint32_t)serialManager.bytesNaSaida();

      // SO DURANTE UM ENVIO. FORA DELE ISSO TRAVA A CAIXA.
      //
      // Sem esta condicao, bytes que sobraram presos na fila (a maquina segue
      // com o pare ativo depois que o envio acabou) mantinham a caixa em
      // PAUSED_FLOW para sempre. E PAUSED nao esta entre os estados que aceitam
      // FILE_BEGIN/OP_START: o proximo programa seria RECUSADO, e o operador
      // veria "comando invalido nesse estado" sem ter feito nada errado.
      //
      // Visto em campo minutos depois de eu instalar esta medida.
      // O ESTADO E CONSULTADO DE 50 EM 50 ms, NAO A CADA VOLTA.
      //
      // Este laco gira a cada 1 ms e dncEstadoAtual() toma o mutex de status.
      // Consultar aqui a cada volta eram mil disputas por segundo pelo mesmo
      // mutex que TaskDNC, TaskWebServer e o protocolo usam -- custo puro, para
      // um valor que muda no maximo algumas vezes por segundo.
      static uint32_t ultimaLeituraEstado = 0;
      static bool     enviandoAgora       = false;
      if (millis() - ultimaLeituraEstado >= 50) {
        ultimaLeituraEstado = millis();
        const DncState e = dncEstadoAtual();
        enviandoAgora = (e == DNC_SENDING) || (e == DNC_DRIP_FEED)
                     || (e == DNC_PAUSED_FLOW);
      }

      if (enviandoAgora && fila > 0 && fila == filaAnterior) {
        if (paradaDesdeMs == 0) paradaDesdeMs = millis();
        if (!avisouParado && millis() - paradaDesdeMs > 200) {
          avisouParado = true;
          setState(DNC_PAUSED_FLOW);
          logEventf("FLOW", "Maquina pediu pausa - %u B seguram na fila "
                            "(freio atuando, nada sai no fio)", (unsigned)fila);
        }
      } else {
        if (avisouParado && fila < filaAnterior) {
          retomarEstadoAposFluxo();
          logEvent("FLOW", "Maquina voltou a consumir - transmissao retomada");
        }
        paradaDesdeMs = 0;
        avisouParado  = false;
      }
      filaAnterior = fila;
    }

    // --- Sinais e permissao --------------------------------------------------
    const bool ctsAgora = (configSerial.flow == FLOW_RTSCTS) ? serialManager.ctsOk() : true;
    if (configSerial.flow == FLOW_RTSCTS) gFluxo.aoMudarCts(ctsAgora);
    gFluxo.definirPausaDoOperador(isUserPaused());

    // --- Com sessao ativa, quem manda e o motor de sessao --------------------
    //
    // Ele nao decide so o tamanho do pedaco: decide tambem QUANDO a transmissao
    // acabou de verdade, o que e a diferenca entre entregar o programa inteiro e
    // cortar o final dele.
    travarMotor();
    const bool sessaoAtiva = gSessao.ativa();
    if (sessaoAtiva) {
      if (configSerial.flow == FLOW_RTSCTS) gSessao.aoMudarCts(ctsAgora);
      gSessao.definirPausaDoOperador(isUserPaused());

      // "Resta" e a fila MAIS o que ainda nao saiu do cartao. Ver a nota em
      // gRestanteNoArquivo: sem a soma, um atraso na leitura do SD faria o
      // motor achar que o programa acabou.
      const int64_t resta    = (int64_t)txRing.available() + (int64_t)gRestanteNoArquivo;
      const int64_t naSaida  = (int64_t)serialManager.bytesNaSaida();

      int pedaco = gSessao.passo(agora, resta, naSaida);
      destravarMotor();

      if (pedaco > (int)sizeof(blocoTx)) pedaco = (int)sizeof(blocoTx);
      bool podeDsr = true;
      if (configSerial.flow == FLOW_DTRDSR) podeDsr = serialManager.dsrOk();

      if (pedaco > 0 && podeDsr && serialManager.isConnected()) {
        const size_t espiados = txRing.peekBlock(blocoTx, (size_t)pedaco);
        const size_t escritos = espiados > 0 ? serialManager.writeBlock(blocoTx, espiados) : 0;
        if (escritos > 0) {
          txRing.discard(escritos);
          travarMotor(); gSessao.aoEscrever(agora, (int64_t)escritos); destravarMotor();

          xSemaphoreTake(gStatusMutex, portMAX_DELAY);
          gStatus.bytesSent += escritos;
          gStatus.lastByteMs = agora;
          xSemaphoreGive(gStatusMutex);
        }
      }

      if (agora - ultimoStatusMs >= 200) {
        ultimoStatusMs = agora;
        updateStatusFlags();
      }
      vTaskDelay(pdMS_TO_TICKS(SERIAL_TASK_DELAY));
      continue;
    }
    destravarMotor();

    // O VIGIA DE CONSUMO NAO ENTRA NA PERMISSAO AINDA, e isso e deliberado.
    //
    // Liga-lo aqui trava: o vigia segura -> sem permissao nao escrevemos -> a
    // UART esvazia -> nao ha mais progresso -> o vigia nunca solta. O envio
    // pararia no meio do programa, que e exatamente a falha que este projeto ja
    // pagou caro para eliminar.
    //
    // Quem sabe soltar essa espera com seguranca e o DncEngine do aplicativo,
    // que ainda nao foi portado. Enquanto isso, a regra de quem pode transmitir
    // continua sendo a que ja funciona na maquina. O `gVigia` existe, esta
    // testado, e entra quando houver a logica de liberacao junto.
    //
    // NAO ligar `definirEsperaDoVigia()` aqui sem portar essa liberacao.

    // O motor cobre pausa, XON/XOFF e CTS. DSR fica de fora dele de proposito
    // -- o motor do aplicativo nunca teve esse modo -- entao entra aqui como
    // condicao adicional, explicita.
    bool permitido = serialManager.isConnected() && gFluxo.envioPermitido();
    if (permitido && configSerial.flow == FLOW_DTRDSR) permitido = serialManager.dsrOk();

    // --- Escrita para a CNC, em pedacos --------------------------------------
    if (permitido) {
      if (!permitidoAntes) {
        permitidoAntes = true;
        ultimoMotivoLogado = 0xFF;
        gAgendador.aoAbrirJanela();
        logEvent("FLOW", "Controle de fluxo liberado");
      }

      const int64_t restantes = (int64_t)txRing.available();
      const int64_t naSaida   = (int64_t)serialManager.bytesNaSaida();

      int pedaco = gAgendador.proximoPedaco(restantes, naSaida);
      if (pedaco > (int)sizeof(blocoTx)) pedaco = (int)sizeof(blocoTx);

      if (pedaco > 0) {
        const size_t espiados = txRing.peekBlock(blocoTx, (size_t)pedaco);
        const size_t escritos = espiados > 0 ? serialManager.writeBlock(blocoTx, espiados) : 0;

        if (escritos > 0) {
          // SO AGORA o dado sai da fila. Se a UART aceitou menos do que
          // espiamos, o resto continua na fila e vai na proxima volta -- em vez
          // de sumir, que era o risco de retirar antes de escrever.
          txRing.discard(escritos);

          gAgendador.aoAceitarEscrita((int64_t)escritos);

          xSemaphoreTake(gStatusMutex, portMAX_DELAY);
          gStatus.bytesSent += escritos;
          gStatus.lastByteMs = agora;
          xSemaphoreGive(gStatusMutex);

          // Entregou o pedaco inteiro sem a maquina reclamar: e o sinal de que
          // da para tentar um pouco mais na proxima. O agendador ainda exige
          // progresso acumulado antes de subir de fato.
          if (escritos == espiados) gAgendador.aoProgredirEstavel();
        }
      }
    } else {
      if (permitidoAntes) {
        permitidoAntes = false;
        gAgendador.aoEntrarEmEspera();
      }
      // SO RECLAMA SE HOUVER O QUE ENVIAR, E SO QUANDO O MOTIVO MUDA.
      //
      // Antes esta mensagem saia a cada 500 ms para sempre, mesmo com a fila
      // vazia. Em poucos minutos ela ocupava as 80 linhas do anel de log e
      // apagava TODO o resto -- inclusive o registro do XOFF que explicava a
      // propria espera. Perder o log e perder a unica ferramenta de diagnostico
      // que temos de longe, e isso aconteceu na bancada.
      //
      // Espera com a fila vazia nao e informacao: nao ha nada sendo segurado.
      const bool haOQueEnviar = (txRing.available() > 0) || (gRestanteNoArquivo > 0);
      const uint8_t motivoAgora = (uint8_t)gFluxo.motivoAtual();

      if (haOQueEnviar && motivoAgora != ultimoMotivoLogado) {
        ultimoMotivoLogado = motivoAgora;
        ultimoLogDeFluxo = agora;
        switch (gFluxo.motivoAtual()) {
        case smi::dnc::motor::MotivoDeEspera::Xoff:
          logEvent("FLOW", "XOFF da CNC: transmissao segurada, aguardando XON"); break;
        case smi::dnc::motor::MotivoDeEspera::CtsBaixo:
          logEvent("FLOW", "CTS baixo: transmissao segurada"); break;
        case smi::dnc::motor::MotivoDeEspera::PausaDoOperador:
          logEvent("FLOW", "Pausado pelo operador"); break;
        default:
          if (configSerial.flow == FLOW_DTRDSR) logEvent("FLOW", "DSR baixo: transmissao segurada");
          break;
        }
      } else if (!haOQueEnviar) {
        // Fila vazia: esquece o motivo, para que a proxima espera de verdade
        // volte a ser registrada.
        ultimoMotivoLogado = 0xFF;
      }
      vTaskDelay(pdMS_TO_TICKS(2));
    }

    // A cada 200 ms, nao a cada volta.
    //
    // `updateStatusFlags()` consulta WiFi.isConnected() e le dois pinos. Fazer
    // isso mil vezes por segundo roubava CPU da tarefa que alimenta a maquina,
    // para atualizar numeros que ninguem le mais que cinco vezes por segundo.
    if (agora - ultimoStatusMs >= 200) {
      ultimoStatusMs = agora;
      updateStatusFlags();
    }

    vTaskDelay(pdMS_TO_TICKS(SERIAL_TASK_DELAY));
  }
}

// ---------- Operacoes DNC ----------
// REMOVIDO: waitTxDrain(timeoutMs).
//
// Esperava a NOSSA fila esvaziar e, no estouro do prazo, encerrava com erro.
// Duas coisas erradas nisso, e as duas apareceram em maquina:
//
//   1. Fila vazia nao e programa entregue. Os bytes ainda estao atravessando o
//      fio e o buffer da CNC. Encerrar ali corta o final do programa.
//   2. Fechar por relogio trata uma maquina lenta como maquina com defeito. Uma
//      CNC que para para processar um bloco pesado nao esta travada.
//
// Quem faz esse trabalho agora e o ControladorDeDrenagem, que adapta o prazo ao
// tamanho e a velocidade, e que ao esgotar o prazo PERGUNTA em vez de fechar.

static void prepareTransferStatus(DncState state, const String& path, uint64_t fileSize) {
  txRing.clear();
  rxRing.clear();
  gXonAllowed = true;

  // O XOFF NAO PODE SOBREVIVER A TRANSFERENCIA ANTERIOR.
  //
  // Muitas CNCs mandam XOFF ao FECHAR a porta, terminado o trabalho. Esse XOFF
  // ficava travado no controlador de fluxo e segurava a transferencia SEGUINTE
  // para sempre, esperando um XON que a maquina nao tinha motivo para mandar.
  // Cada transferencia comeca do zero.
  travarMotor();
  gFluxo.reiniciar();
  destravarMotor();
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  gStatus.state = state;
  gStatus.resumeState = state;
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
  const DncSerialConfig configSerial = copiarConfigSerial();
  prepareTransferStatus(dripMode ? DNC_DRIP_FEED : DNC_SENDING, path, size);
  logEventf("INFO", "%s iniciado: %s (%llu bytes)", dripMode ? "Drip feed" : "Envio", path.c_str(), size);

  uint8_t block[SD_READ_BLOCK];
  uint64_t readTotal = 0;

  // Abre a sessao ANTES de empurrar qualquer byte, para que a TaskSerial ja
  // encontre o motor no comando quando o primeiro pedaco aparecer na fila.
  gRestanteNoArquivo = (uint32_t)size;
  travarMotor();
  gConfigMotor = traduzirConfigDoMotor(configSerial);
  gSessao.definirConfig(gConfigMotor);
  gSessao.iniciarEnvio(millis(), (int64_t)size);
  destravarMotor();

  xSemaphoreTake(storage.mutex(), portMAX_DELAY);
  File f = SD.open(path, FILE_READ);
  const bool abriu = f && !f.isDirectory();
  if (!abriu && f) f.close();
  xSemaphoreGive(storage.mutex());

  if (!abriu) {
    gRestanteNoArquivo = 0;
    travarMotor(); gSessao.abortar(millis()); destravarMotor();
    setError("Falha ao abrir arquivo para leitura");
    return;
  }

  bool cancelado = false;

  while (true) {
    // ALIMENTAR O WATCHDOG AQUI E OBRIGATORIO.
    //
    // Esta funcao roda DENTRO da TaskDNC, que e vigiada. O reset so acontecia
    // quando ela voltava ao laco principal da tarefa -- e um envio de 14 KB a
    // 9600 baud leva mais de 15 segundos sem voltar. Resultado: o equipamento
    // reiniciava no meio da transferencia. Comprovado em bancada.
    esp_task_wdt_reset(); baterPonto(TAREFA_DNC);

    if (isCancelRequested()) { cancelado = true; logEvent("WARN", "Transferencia cancelada pelo operador"); break; }

    // O CARTAO E SOLTO ENTRE OS BLOCOS, e isso nao e detalhe.
    //
    // A versao anterior segurava o mutex do SD do inicio ao fim da
    // transferencia. Agora a sessao espera a confirmacao do operador, o que pode
    // levar minutos -- e nesse tempo o portal e a TaskStorage ficariam
    // congelados. Solto a cada bloco.
    size_t n = 0;
    bool acabou = false;
    xSemaphoreTake(storage.mutex(), portMAX_DELAY);
    if (f.available()) {
      n = f.read(block, sizeof(block));
    } else {
      acabou = true;
    }
    xSemaphoreGive(storage.mutex());

    if (acabou) break;
    if (n == 0) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }

    readTotal += n;

    size_t pushed = 0;
    while (pushed < n) {
      if (isCancelRequested()) { cancelado = true; break; }
      size_t p = 0;
      txRing.pushBlock(block + pushed, n - pushed, &p);
      pushed += p;
      if (p == 0) {
        // ALIMENTAR O CAO DE GUARDA AQUI DENTRO E OBRIGATORIO.
        //
        // ESTE FOI O DEFEITO QUE REINICIAVA A CAIXA NO MEIO DA CARGA DE NCK.
        //
        // Quando a maquina freia (CTS baixo ou XOFF), a TaskSerial para de
        // escrever, a fila enche, e  passa a devolver zero. Esta
        // espera e CORRETA -- e o freio funcionando. So que o laco externo e
        // quem alimentava o cao de guarda, e nunca mais se chegava nele: a
        // TaskDNC ficava presa aqui, girando de 2 em 2 ms, calada.
        //
        // Aos 15 segundos o watchdog abortava e o equipamento reiniciava no
        // meio da transmissao. A maquina via o fluxo morrer de repente e
        // acusava erro -- na Siemens, "overrun buffer".
        //
        // A perversidade: quanto MELHOR a maquina freia, mais rapido a caixa
        // reiniciava. Freio longo, como a Siemens gravando dado de maquina em
        // memoria nao volatil, era reinicio garantido.
        //
        // Esperar segurando a fila e o comportamento certo. O que faltava era
        // dizer ao vigia que estamos vivos enquanto esperamos.
        esp_task_wdt_reset(); baterPonto(TAREFA_DNC);
        vTaskDelay(pdMS_TO_TICKS(2));
      }
    }
    if (cancelado) break;

    // Desconta so o que ja esta na fila. Enquanto sobrar arquivo, este numero
    // impede a TaskSerial de comecar a drenagem cedo demais.
    gRestanteNoArquivo = (uint32_t)(size - readTotal);

    if ((readTotal % 8192) < SD_READ_BLOCK) {
      logEventf("INFO", "%llu/%llu bytes lidos do SD", readTotal, size);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  xSemaphoreTake(storage.mutex(), portMAX_DELAY);
  f.close();
  xSemaphoreGive(storage.mutex());

  if (cancelado) {
    gRestanteNoArquivo = 0;
    txRing.clear();
    travarMotor(); gSessao.abortar(millis()); destravarMotor();
    setState(DNC_CONNECTED);
    return;
  }

  gRestanteNoArquivo = 0;
  logEventf("INFO", "Arquivo lido por completo (%llu bytes). Aguardando a maquina consumir.", readTotal);

  // --- Espera a sessao decidir ---------------------------------------------
  //
  // Daqui em diante quem manda e o motor: ele decide quando a maquina realmente
  // terminou de receber. Esta tarefa so acompanha, publica os avisos e obedece
  // ao operador. NAO ha prazo maximo aqui de proposito -- fechar por relogio e
  // exatamente o que deixava programa pela metade na maquina.
  bool avisouTravada = false;
  while (true) {
    // Este laco pode durar o quanto a maquina precisar -- nao ha prazo maximo,
    // de proposito. Sem alimentar o watchdog, "esperar a maquina" viraria
    // "reiniciar o equipamento", que e o oposto do que esta espera existe para
    // garantir.
    esp_task_wdt_reset(); baterPonto(TAREFA_DNC);

    if (isCancelRequested()) {
      travarMotor(); gSessao.abortar(millis()); destravarMotor();
      txRing.clear();
      logEvent("WARN", "Encerrado pelo operador durante a drenagem final");
      setState(DNC_CONNECTED);
      return;
    }

    travarMotor();
    const smi::dnc::motor::EstadoDaSessao est = gSessao.estado();
    const int64_t pendentes = gSessao.bytesPendentes();
    smi::dnc::motor::AvisoDaSessao aviso = gSessao.consumirAviso();
    destravarMotor();

    while (aviso != smi::dnc::motor::AvisoDaSessao::Nenhum) {
      switch (aviso) {
      case smi::dnc::motor::AvisoDaSessao::DrenagemIniciada:
        logEvent("INFO", "Drenagem final iniciada: aguardando a maquina consumir o restante.");
        break;
      case smi::dnc::motor::AvisoDaSessao::DrenagemTravada:
        avisouTravada = true;
        setState(DNC_DRAIN_STALLED);
        logEventf("WARN", "A maquina parou de consumir com %lld bytes pendentes. "
                          "PORTA MANTIDA ABERTA. Use 'continuar' para seguir aguardando "
                          "ou 'cancelar' para encerrar.", (long long)pendentes);
        break;
      case smi::dnc::motor::AvisoDaSessao::AvisoDeDrenagemLimpo:
        avisouTravada = false;
        setState(dripMode ? DNC_DRIP_FEED : DNC_SENDING);
        logEvent("INFO", "A maquina voltou a consumir; aviso retirado.");
        break;
      case smi::dnc::motor::AvisoDaSessao::ProntoParaFinalizar:
        setState(DNC_AWAIT_FINISH);
        logEventf("OK", "Programa transferido por completo (%llu bytes). Confirme o "
                        "encerramento somente quando a maquina tiver concluido a execucao.",
                  readTotal);
        break;
      default: break;
      }
      travarMotor(); aviso = gSessao.consumirAviso(); destravarMotor();
    }

    if (est == smi::dnc::motor::EstadoDaSessao::Concluida) {
      logEvent("OK", dripMode ? "Drip feed concluido" : "Envio concluido");
      setState(DNC_CONNECTED);
      return;
    }
    if (est == smi::dnc::motor::EstadoDaSessao::Abortada) {
      txRing.clear();
      setState(DNC_CONNECTED);
      return;
    }
    if (est != smi::dnc::motor::EstadoDaSessao::AguardandoOperadorFinalizar
        && !avisouTravada
        && !isUserPaused()) {
      setState(dripMode ? DNC_DRIP_FEED : DNC_SENDING);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

static void receiveFromCnc(const String& requestedPath) {
  String path = requestedPath.length() > 1 ? requestedPath : autoReceiveName();
  const DncSerialConfig configSerial = copiarConfigSerial();
  const uint32_t overflowInicial = gRxOverflowCount;
  prepareTransferStatus(DNC_RECEIVING, path, 0);
  logEventf("INFO", "Recepcao iniciada: %s", path.c_str());

  uint8_t block[SD_WRITE_BLOCK];
  uint32_t lastRx = millis();
  uint64_t written = 0;

  // Verdadeiro enquanto ha uma pergunta na tela sobre a pausa da maquina.
  bool aguardandoOperador = false;

  // QUEM RECEBE MANDA O XON. Isto faltava por completo.
  //
  // No sentido da recepcao o XON/XOFF se inverte: e o RECEPTOR que diz "pode
  // transmitir". Muitas CNCs ficam paradas esperando esse caractere, e o
  // sintoma e exatamente o que apareceu na bancada -- a maquina declara o
  // envio concluido e chegam ZERO bytes.
  const bool usaXonXoff = (configSerial.flow == FLOW_XONXOFF);
  bool seguramosAMaquina = false;
  uint32_t ultimoXonMs = 0;
  bool cancelada = false;
  bool falhaRecepcao = false;
  String erroRecepcao;

  if (usaXonXoff) {
    serialManager.enviarControleDeFluxo(0x11);
    ultimoXonMs = millis();
    logEvent("FLOW", "XON enviado: liberando a maquina para transmitir");
  }

  xSemaphoreTake(storage.mutex(), portMAX_DELAY);
  File f = SD.open(path, FILE_WRITE);
  const bool abriu = (bool)f;
  xSemaphoreGive(storage.mutex());
  if (!abriu) {
    setError("Falha ao abrir arquivo de recepcao");
    return;
  }

  for (;;) {
    // Mesmo motivo do envio: esta funcao roda dentro da TaskDNC, que e vigiada,
    // e uma recepcao demora muito mais que o prazo do watchdog.
    esp_task_wdt_reset(); baterPonto(TAREFA_DNC);

    if (isCancelRequested()) {
      logEvent("WARN", "Recepcao cancelada pelo operador");
      cancelada = true;
      break;
    }

    if (gRxOverflowCount != overflowInicial) {
      if (usaXonXoff) serialManager.enviarControleDeFluxo(0x13);
      falhaRecepcao = true;
      erroRecepcao = "Recepcao interrompida: buffer RX transbordou";
      break;
    }

    // SEGURAR A MAQUINA ANTES DE PERDER DADO.
    //
    // Sem isto, uma CNC que manda mais rapido do que gravamos no cartao encheria
    // o anel e o excedente seria descartado em silencio -- programa recebido
    // pela metade, sem erro nenhum. E o espelho exato do transbordo que evitamos
    // no envio, e ate agora nao existia deste lado.
    if (usaXonXoff) {
      const size_t livre = rxRing.freeSpace();
      const size_t capacidade = rxRing.capacity();
      if (!seguramosAMaquina && livre < capacidade / 4) {
        serialManager.enviarControleDeFluxo(0x13);          // XOFF
        seguramosAMaquina = true;
        logEvent("FLOW", "XOFF enviado: buffer de recepcao quase cheio");
      } else if (seguramosAMaquina && livre > capacidade / 2) {
        serialManager.enviarControleDeFluxo(0x11);          // XON
        seguramosAMaquina = false;
        ultimoXonMs = millis();
        logEvent("FLOW", "XON enviado: buffer liberado");
      }
    }

    size_t n = rxRing.popBlock(block, sizeof(block));
    if (n > 0) {
      // O CARTAO E SOLTO ENTRE OS BLOCOS.
      //
      // Segurar o mutex do SD pela recepcao inteira travava a TaskWebServer na
      // primeira consulta de espaco livre -- e como o protocolo do aplicativo
      // roda NA MESMA TAREFA, ele parava junto. Os PINGs do aplicativo ficavam
      // sem ser lidos, o detector de cliente morto concluia que ele havia
      // sumido, e a caixa derrubava uma conexao viva. Quanto mais longo o
      // programa, mais vezes acontecia.
      //
      // O envio ja tinha sido corrigido assim hoje; a recepcao ficou para tras.
      // Insiste no que o cartao nao aceitou de primeira: escrita curta e
      // normal e nao pode abortar a captacao do programa. Ver escreverTudo().
      const size_t gravados = StorageManager::escreverTudo(f, block, n,
                                                           storage.mutex());

      written += gravados;
      lastRx = millis();
      xSemaphoreTake(gStatusMutex, portMAX_DELAY);
      gStatus.fileSize = written;
      if (gravados != n) ++gStatus.sdWriteFailures;
      xSemaphoreGive(gStatusMutex);

      if (gravados != n) {
        if (usaXonXoff) serialManager.enviarControleDeFluxo(0x13);
        falhaRecepcao = true;
        erroRecepcao = "Falha ao gravar no microSD (cartao cheio ou removido)";
        logEventf("ERRO", "microSD gravou somente %u de %u bytes",
                  (unsigned)gravados, (unsigned)n);
        break;
      }
    } else {
      // XON REPETIDO ENQUANTO NADA CHEGA.
      //
      // Um XON perdido no ruido da linha deixaria a maquina esperando para
      // sempre, e o operador olhando uma tela parada sem explicacao. Repetir a
      // cada 2 s enquanto nao chegou nada custa um caractere e resolve.
      if (usaXonXoff && written == 0 && !seguramosAMaquina
          && millis() - ultimoXonMs > 2000) {
        serialManager.enviarControleDeFluxo(0x11);
        ultimoXonMs = millis();
      }

      // SILENCIO NAO E FIM. A PORTA FICA ABERTA E QUEM DECIDE E O OPERADOR.
      //
      // Antes: `if (silencio > timeoutMs) break;` -- tres segundos parados e a
      // porta fechava. Parametros de Siemens saem com pausas longas no meio, e
      // a maquina chega a ser desligada durante o processo: a primeira pausa
      // encerrava a recepcao e o resto se perdia, com "sucesso" na tela.
      //
      // Agora e o espelho do que o envio ja fazia: avisa, mantem a porta aberta
      // e a maquina liberada com XON, e espera a resposta. Nada e descartado.
      if (written > 0 && !aguardandoOperador
          && millis() - lastRx > configSerial.timeoutMs) {
        aguardandoOperador = true;
        gRecepcaoContinuar = false;
        gRecepcaoFinalizar = false;
        setState(DNC_RECEIVE_IDLE);
        logEventf("WARN", "Maquina parou de enviar ha %lu s. Porta ABERTA aguardando decisao.",
                  (unsigned long)(configSerial.timeoutMs / 1000));
      }

      if (aguardandoOperador) {
        // Voltou a mandar sozinha: a pergunta nao vale mais, retoma calado.
        if (millis() - lastRx < configSerial.timeoutMs) {
          aguardandoOperador = false;
          setState(DNC_RECEIVING);
          logEvent("OK", "A maquina voltou a enviar. Recepcao retomada.");
        } else if (gRecepcaoFinalizar) {
          logEvent("INFO", "Operador encerrou a recepcao. Guardando o que chegou.");
          break;
        } else if (gRecepcaoContinuar) {
          // Reabre o prazo: sem isto a pergunta voltaria no instante seguinte.
          gRecepcaoContinuar = false;
          lastRx = millis();
          aguardandoOperador = false;
          setState(DNC_RECEIVING);
          logEvent("INFO", "Operador escolheu continuar aguardando a maquina.");
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  xSemaphoreTake(storage.mutex(), portMAX_DELAY);
  f.flush();
  f.close();
  xSemaphoreGive(storage.mutex());

  // RECEPCAO SEM NENHUM BYTE NAO DEIXA ARQUIVO.
  //
  // O arquivo e criado ANTES de a maquina mandar qualquer coisa -- e se ela nao
  // mandar nada (operador encerrou antes, cabo no lugar errado, maquina em modo
  // errado), o que sobra no cartao e um arquivo de ZERO BYTES.
  //
  // Visto em campo: "Mach9.nc  0 B" na lista de recebidos. Pior que ocupar
  // espaco, ele MENTE: aparece junto dos programas bons e da a entender que a
  // captacao funcionou. Quem for usar so descobre na maquina.
  if (written == 0) {
    xSemaphoreTake(storage.mutex(), portMAX_DELAY);
    const bool removido = SD.remove(path);
    xSemaphoreGive(storage.mutex());
    logEventf("WARN", "Recepcao sem nenhum byte: arquivo vazio %s %s",
              path.c_str(), removido ? "descartado" : "NAO pode ser descartado");
  }

  if (falhaRecepcao) {
    setError(erroRecepcao.c_str());
    return;
  }

  if (cancelada) {
    setState(DNC_CONNECTED);
    return;
  }

  logEventf("OK", "Recepcao finalizada: %s (%llu bytes)", path.c_str(), written);

  // NAO volta para DNC_CONNECTED aqui.
  //
  // CONNECTED nao e publicado (ver estadoDoProtocolo), entao terminar assim
  // deixava o aplicativo sem nenhum aviso de que a recepcao acabou. Quem
  // publica e a TaskWebServer, no passo dela -- publicar direto desta tarefa
  // escreveria no mesmo socket que ela usa para mandar o arquivo, e os quadros
  // se embaralhariam. O estado volta para CONNECTED no proximo comando.
  setState(DNC_RECEIVE_DONE);
}

// ---------- TaskDNC: orquestra comandos ----------
void TaskDNC(void* pv) {
  const uint8_t meuIndice = TAREFA_DNC;
  strncpy(gNomeDaTarefa[TAREFA_DNC], "DNC", 11);
  gNomeDaTarefa[TAREFA_DNC][11] = 0;
  setupWatchdogForCurrentTask();
  logEvent("INFO", "TaskDNC iniciada");
  DncCommand cmd;

  for (;;) {
    esp_task_wdt_reset(); baterPonto(meuIndice);
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
        {
          const DncSerialConfig configSerial = copiarConfigSerial();
          serialManager.applyConfig(configSerial);
          setState(DNC_CONNECTED);
          logEventf("INFO", "Serial aplicado: %lu %u%s%u %s",
                    (unsigned long)configSerial.baudRate, configSerial.dataBits,
                    parityToText(configSerial.parity), configSerial.stopBits,
                    flowToText(configSerial.flow));
          break;
        }

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
          solicitarPausaDnc("fila DNC");
          break;

        case CMD_RESUME:
          solicitarRetomadaDnc("fila DNC");
          break;

        case CMD_CANCEL:
          solicitarCancelamentoDnc("fila DNC");
          break;

        default:
          break;
      }
    }
  }
}

// ---------- TaskStorage: diagnostico e manutencao do SD ----------
void TaskStorage(void* pv) {
  const uint8_t meuIndice = TAREFA_STORAGE;
  strncpy(gNomeDaTarefa[TAREFA_STORAGE], "STORAGE", 11);
  gNomeDaTarefa[TAREFA_STORAGE][11] = 0;
  setupWatchdogForCurrentTask();
  logEvent("INFO", "TaskStorage iniciada");
  uint32_t proximaTentativa = 0;
  uint32_t esperaDaMontagem = 5000;
  bool jaAvisouFalha = false;
  for (;;) {
    esp_task_wdt_reset(); baterPonto(meuIndice);

    // Comparacao por subtracao com sinal: sobrevive a volta do millis() aos 49
    // dias. Um equipamento de painel fica ligado meses.
    if (!storage.isOk() && (int32_t)(millis() - proximaTentativa) >= 0) {
      // SAI DO WATCHDOG SO DURANTE A TENTATIVA.
      //
      // `storage.begin()` pode ficar dezenas de segundos tentando falar com um
      // cartao que nao responde. Se esta tarefa continuasse vigiada, o watchdog
      // reiniciaria o equipamento inteiro por causa de um cartao mal encaixado
      // -- exatamente o defeito que tirei do setup(). Fora da tentativa ela
      // volta a ser vigiada normalmente.
      esp_task_wdt_delete(NULL);
      const bool ok = storage.begin();
      esp_task_wdt_add(NULL);
      esp_task_wdt_reset(); baterPonto(meuIndice);

      if (ok) {
        jaAvisouFalha = false;
        logEvent("OK", "microSD montado");
      } else if (!jaAvisouFalha) {
        // Uma vez, nao a cada tentativa: senao o aviso empurra as mensagens
        // uteis para fora do anel de logs recentes.
        jaAvisouFalha = true;
        logEvent("WARN", "microSD nao respondeu. Segue tentando em segundo plano.");
      }
      // ESPACAR AS TENTATIVAS QUANDO NAO HA CARTAO.
      //
      // Cinco segundos fixos significa remontar o driver 12 vezes por minuto
      // para sempre quando ninguem colocou cartao -- e cada tentativa e uma
      // janela em que o cartao fica ocupado. Quem acabou de colocar o cartao
      // espera pouco; quem nunca vai colocar para de pagar o preco.
      if (ok) {
        esperaDaMontagem = 5000;
      } else if (esperaDaMontagem < 60000) {
        esperaDaMontagem = esperaDaMontagem * 2;
        if (esperaDaMontagem > 60000) esperaDaMontagem = 60000;
      }
      proximaTentativa = millis() + esperaDaMontagem;
    }
    xSemaphoreTake(gStatusMutex, portMAX_DELAY);
    gStatus.sdOk = storage.isOk();
    xSemaphoreGive(gStatusMutex);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ---------- TaskNetwork: reconexao Wi-Fi ----------
void TaskNetwork(void* pv) {
  const uint8_t meuIndice = TAREFA_NETWORK;
  strncpy(gNomeDaTarefa[TAREFA_NETWORK], "NETWORK", 11);
  gNomeDaTarefa[TAREFA_NETWORK][11] = 0;
  setupWatchdogForCurrentTask();
  logEvent("INFO", "TaskNetwork iniciada");
  uint32_t lastTry = 0;
  uint32_t foraDesde = 0;   // desde quando o Wi-Fi esta fora; 0 = esta no ar
  for (;;) {
    esp_task_wdt_reset(); baterPonto(meuIndice);

    // TROCA DE REDE PEDIDA PELO APLICATIVO.
    //
    // Aplicada AQUI, e nao no tratador do protocolo, porque entrar na rede nova
    // derruba o ponto de acesso de socorro -- e com ele a conexao por onde o
    // pedido chegou. Respondendo primeiro e trocando depois, o aplicativo
    // recebe a confirmacao e so entao perde a conexao, que e o esperado.
    if (gTrocaDeRedePedida) {
      gTrocaDeRedePedida = false;
      logEventf("INFO", "Entrando na rede '%s' cadastrada pelo aplicativo",
                gRedeNovaSsid.c_str());
      WiFi.disconnect(false, false);
      delay(100);
      WiFi.begin(gRedeNovaSsid.c_str(), gRedeNovaSenha.c_str());
      lastTry = millis();
      foraDesde = millis();
    }

    const NetworkConfig rede = copiarConfigRede();
    const wifi_mode_t modo = WiFi.getMode();
    const bool tentandoEstacao = (modo == WIFI_STA || modo == WIFI_AP_STA);

    // NAO BRIGAR COM A RECONEXAO AUTOMATICA DA PILHA.
    //
    // `WiFi.disconnect()` seguido de `begin()` a cada volta ATRAPALHA: a pilha
    // ja esta tentando reassociar sozinha, e derrubar a tentativa dela no meio
    // ALONGA a queda em vez de encurtar. Uma piscada de dois segundos virava
    // dezenas -- tempo de sobra para cortar uma transferencia.
    //
    // Entao: deixa a pilha trabalhar, e so intervem se ela nao tiver resolvido
    // depois de MUITO tempo, que e quando de fato ha algo travado.
    if (tentandoEstacao && WiFi.status() != WL_CONNECTED && rede.ssid.length()) {
      if (foraDesde == 0) foraDesde = millis();

      if (millis() - foraDesde > WIFI_RECONNECT_MS && millis() - lastTry > WIFI_RECONNECT_MS) {
        lastTry = millis();
        WiFi.begin(rede.ssid.c_str(), rede.password.c_str());
        logEventf("WARN", "Wi-Fi fora ha %lu s. Forcando nova tentativa.",
                  (unsigned long)((millis() - foraDesde) / 1000));
      }
    } else if (WiFi.status() == WL_CONNECTED) {
      if (foraDesde != 0) {
        logEventf("OK", "Wi-Fi de volta apos %lu s (sinal %d dBm)",
                  (unsigned long)((millis() - foraDesde) / 1000), (int)WiFi.RSSI());
      }
      foraDesde = 0;
    }

    // VOLTOU PARA A REDE DA FABRICA: DESLIGA O AP DE SOCORRO.
    //
    // Deixar o ponto de acesso no ar depois de reconectar poluiria a lista de
    // redes da fabrica com uma SMI_DNC_ESP32 por equipamento -- e a caixa ficaria
    // anunciando um caminho que ninguem deve usar quando o normal esta de pe.
    if (modo == WIFI_AP_STA && WiFi.status() == WL_CONNECTED) {
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
      logEventf("INFO", "Entrou na rede '%s' IP %s. AP de socorro desligado.",
                rede.ssid.c_str(), WiFi.localIP().toString().c_str());
    }
    // SINAL DE TEMPOS EM TEMPOS.
    //
    // Uma queda isolada nao diz nada; a serie diz. Se o sinal ja vinha ruim
    // antes de cair, o problema e alcance ou interferencia -- e a solucao e
    // antena/posicao, nao firmware.
    // DE CINCO EM CINCO MINUTOS, OU QUANDO MUDA DE PATAMAR.
    //
    // A cada 30 s esta linha sozinha enchia as posicoes do historico. Quando a
    // caixa da Polaris reiniciou no meio de um envio, o que sobrou no log era SO
    // isto -- nenhuma pista da falha. Medida de rotina nao pode apagar registro
    // de problema.
    static uint32_t ultimoSinalMs = 0;
    static int      ultimoRssi    = 0;
    static uint32_t ultimaHeap    = 0;
    const int      rssiAgora = (int)WiFi.RSSI();
    const uint32_t heapAgora = (uint32_t)ESP.getFreeHeap();
    const bool mudouDePatamar = (abs(rssiAgora - ultimoRssi) >= 8)
                             || (ultimaHeap > 0 && heapAgora + 8192 < ultimaHeap);
    if (WiFi.status() == WL_CONNECTED
        && (millis() - ultimoSinalMs > 300000
            || (mudouDePatamar && millis() - ultimoSinalMs > 30000))) {
      ultimoSinalMs = millis();
      ultimoRssi    = rssiAgora;
      ultimaHeap    = heapAgora;
      // MEMORIA LIVRE JUNTO DO SINAL.
      //
      // A caixa apareceu com 104 BYTES livres depois de algum tempo no ar, e o
      // mDNS comecou a falhar por falta de memoria. Sem uma serie temporal nao
      // da para separar VAZAMENTO (cai devagar e nao volta) de uma alocacao
      // grande e passageira (cai e volta).
      //
      // `minima` e o menor valor ja visto desde o boot: e ele que denuncia um
      // pico que passou despercebido entre duas medidas.
      logEventf("INFO", "Sinal do Wi-Fi: %d dBm (canal %u) | livre %u B, minima %u B, maior bloco %u B",
                (int)WiFi.RSSI(), (unsigned)WiFi.channel(),
                (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getMinFreeHeap(),
                (unsigned)ESP.getMaxAllocHeap());
    }

    // Se o Wi-Fi so subiu depois do setup, o OTA entra agora.
    iniciarOta();
    updateStatusFlags();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ---------- TaskWebServer ----------
void TaskWebServer(void* pv) {
  const uint8_t meuIndice = TAREFA_WEB;
  strncpy(gNomeDaTarefa[TAREFA_WEB], "WEB", 11);
  gNomeDaTarefa[TAREFA_WEB][11] = 0;

  // ESTA TAREFA NAO ENTRA NO CAO DE GUARDA. MEDIDO EM CAMPO, DUAS VEZES.
  //
  // O QUE ACONTECIA: a caixa reiniciava no meio de uma usinagem de 7,9 MB. O
  // relatorio de boot apontou, sem ambiguidade:
  //
  //     Tarefa que travou: WEB (parada ha 59206 ms)
  //       SERIAL: atraso 0 ms · DNC: 1 ms · STORAGE: 999 ms · NETWORK: 811 ms
  //       WEB: 59206 ms
  //
  // Todas as outras em dia; so esta presa, por quase um minuto. A causa e
  // servir o historico de log: `sendContent` BLOQUEIA quando a janela TCP
  // enche, e com o radio disputado pela transmissao isso trava por dezenas de
  // segundos. Alimentar o cao a cada linha enviada -- que foi a primeira
  // tentativa -- nao resolve: o bloqueio acontece DENTRO de uma escrita, e o
  // reset entre linhas nunca chega a rodar.
  //
  // POR QUE TIRAR E O CERTO, E NAO UM PALIATIVO: o cao existe para garantir que
  // a maquina nao fique sem ser alimentada. Quem alimenta sao TaskSerial e
  // TaskDNC, e elas continuam vigiadas. Esta aqui serve paginas e o protocolo
  // do aplicativo -- travar aqui atrasa uma tela, nao corta programa. Reiniciar
  // o equipamento inteiro no meio de 7,9 MB por causa de uma pagina aberta num
  // navegador e trocar um incomodo por um estrago.
  //
  // O preco: se esta tarefa travar de vez, o portal e o aplicativo param de
  // responder ate alguem reiniciar. Fica VISIVEL -- o operador ve a tela
  // parada --, ao contrario do reinicio, que acontecia calado no meio da noite
  // e so aparecia como programa cortado na maquina.
  //
  // NAO chamar setupWatchdogForCurrentTask() aqui e a correcao inteira.
  logEvent("INFO", "TaskWebServer iniciada (fora do cao de guarda, por desenho)");
  for (;;) {
    // Sem esp_task_wdt_reset(): esta tarefa NAO esta registrada no cao -- ver a
    // nota no inicio dela. A batida de ponto CONTINUA, porque e ela que diz,
    // depois de um reinicio por outra causa, se esta aqui tambem estava presa.
    baterPonto(meuIndice);
    webPortal.loop();
    // O OTA vive aqui, e nao na TaskNetwork, porque esta tarefa gira a cada
    // 2 ms e aquela a cada 1 s. O protocolo do OTA nao tolera um segundo de
    // atraso entre pacotes. Enquanto grava, o portal web para de responder --
    // e isso e esperado: sao dois minutos, e a alternativa seria gravar
    // firmware em pedacos entre requisicoes HTTP.
    gOta.laco();

    // O protocolo do aplicativo tambem gira aqui, pelo mesmo motivo do OTA:
    // esta tarefa roda a cada 2 ms, e um comando de operador nao pode esperar
    // um segundo para ser atendido.
    gProtocolo.laco();
    gDescoberta.laco();

    // ESTADO DAS LINHAS RS-232 PARA O APLICATIVO.
    //
    // `publicarSinais` existia e NUNCA era chamada: o painel de linhas do
    // aplicativo ficava com "?" em tudo e a mensagem "leitura travou". Na
    // pratica o operador nao tinha como ver se a maquina esta pedindo para
    // parar -- e e exatamente isso que separa "a maquina freia e nos
    // ignoramos" de "a maquina nao freia", que foi a duvida que custou uma
    // tarde na carga de parametros da Siemens.
    //
    // Duas vezes por segundo basta para o olho e nao pesa na rede.
    {
      static uint32_t ultimoSinaisMs = 0;
      if (millis() - ultimoSinaisMs >= 500) {
        ultimoSinaisMs = millis();
        uint8_t linhas = 0;
        if (serialManager.ctsOk()) linhas |= 0x01;   // CTS
        if (serialManager.dsrOk()) linhas |= 0x02;   // DSR
        if (serialManager.isConnected()) linhas |= 0x10 | 0x20;  // RTS e DTR, nossas saidas
        gProtocolo.publicarSinais(linhas);
      }
    }

    // Estado e progresso para o aplicativo. O proprio servidor limita o
    // progresso a um evento a cada 200 ms -- encher o Wi-Fi de mensagens
    // roubaria CPU da tarefa que alimenta a maquina.
    {
      SystemStatus s;
      xSemaphoreTake(gStatusMutex, portMAX_DELAY);
      memcpy(&s, &gStatus, sizeof(s));
      xSemaphoreGive(gStatusMutex);

      // TRADUZIDO, nunca o enum interno cru. Ver estadoDoProtocolo().
      const uint8_t estadoPublicavel = estadoDoProtocolo(s.state);
      static uint8_t ultimoEstadoPublicado = kNaoPublicar;
      if (estadoPublicavel != kNaoPublicar && estadoPublicavel != ultimoEstadoPublicado) {
        ultimoEstadoPublicado = estadoPublicavel;
        gProtocolo.publicarEstado(estadoPublicavel);
      }
      if (estadoPublicavel == kNaoPublicar) {
        // Esquece o ultimo publicado: quando a proxima operacao comecar, o
        // estado precisa ser publicado de novo mesmo que seja igual ao de antes.
        ultimoEstadoPublicado = kNaoPublicar;
      }
      // O PROGRESSO SEGUE O SENTIDO DA OPERACAO.
      //
      // Publicar sempre `bytesSent` deixava a tela morta durante a RECEPCAO --
      // ali esse contador fica em zero e quem cresce e `bytesReceived`. O
      // operador via a caixa gravando o programa e o aplicativo parado, sem
      // saber se estava funcionando.
      //
      // Na recepcao o total e DESCONHECIDO: so a maquina sabe o tamanho do
      // programa, e ela nao informa. Mando total = 0 de proposito, para a tela
      // mostrar a contagem de bytes sem inventar uma porcentagem.
      if (s.state == DNC_RECEIVING) {
        gProtocolo.publicarProgresso(s.bytesReceived, 0, s.bytesPerSecond);
      } else if (s.fileSize > 0) {
        gProtocolo.publicarProgresso(s.bytesSent, s.fileSize, s.bytesPerSecond);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ---------- TaskLogger ----------
void TaskLogger(void* pv) {
  const uint8_t meuIndice = TAREFA_LOGGER;
  strncpy(gNomeDaTarefa[TAREFA_LOGGER], "LOGGER", 11);
  gNomeDaTarefa[TAREFA_LOGGER][11] = 0;
  setupWatchdogForCurrentTask();
  LogMessage msg;
  for (;;) {
    esp_task_wdt_reset(); baterPonto(meuIndice);
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

// CH376 FORA DO BARRAMENTO, ANTES DE QUALQUER LOG.
//
// Na placa de producao o CS e o INT do CH376 estao nos pinos da UART 0, que e
// onde o sistema escreve o log. Enquanto o pino for da UART, cada mensagem
// liga e desliga o CS do CH376 -- e se isso acontece com o cartao sendo lido,
// os dois chips falam ao mesmo tempo no MISO. Tomando o pino como GPIO comum,
// a UART 0 perde o caminho ate ele: o log do sistema simplesmente nao sai mais
// por ali (o nosso sai pela USB do C5). No prototipo nao ha CH376: nada a fazer.
static void prenderCh376ForaDoBarramento() {
#ifdef PIN_CH376_CS
  pinMode(PIN_CH376_CS, OUTPUT);
  digitalWrite(PIN_CH376_CS, HIGH);
  pinMode(PIN_CH376_INT, INPUT_PULLUP);
#endif
#ifdef PIN_LED_ALARME
  pinMode(PIN_LED_ALARME, OUTPUT);
  digitalWrite(PIN_LED_ALARME, LOW);
#endif
}

void setup() {
  // PRIMEIRA COISA DO SETUP: salvar o que o boot anterior deixou.
  gTinhaMarcasValidas = (gMarcaValida == MARCA_VALIDA);
  if (gTinhaMarcasValidas) {
    for (uint8_t i = 0; i < TAREFA_MAX; i++) {
      gBatidaNoBootAnterior[i] = gBatidaDaTarefa[i];
      memcpy(gNomeNoBootAnterior[i], gNomeDaTarefa[i], sizeof(gNomeNoBootAnterior[i]));
    }
  }
  // A partir daqui as marcas valem para ESTE boot.
  gMarcaValida = MARCA_VALIDA;
  for (uint8_t i = 0; i < TAREFA_MAX; i++) { gBatidaDaTarefa[i] = 0; gNomeDaTarefa[i][0] = 0; }

  prenderCh376ForaDoBarramento();

  Serial.begin(115200);
  delay(500);
  Serial.println("\nSMI ESP32 DNC Industrial iniciando...");
  Serial.printf("VERSAO: %s  (compilado em %s)\n", FIRMWARE_VERSION, FIRMWARE_BUILD);
  Serial.printf("PLACA: %s  chip %s rev %d, %d nucleo(s)\n", PLACA_NOME,
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores());
  Serial.printf("PARTICAO EM USO: %s\n", particaoEmUso().c_str());

  // POR QUE A CAIXA REINICIOU. SEM ISTO NAO HA DIAGNOSTICO POSSIVEL.
  //
  // A caixa da Polaris reiniciou no meio de um envio e levou o log junto -- e
  // nao havia UMA LINHA dizendo o motivo. Watchdog, falta de energia e excecao
  // de software pedem consertos completamente diferentes, e sem este dado a
  // escolha entre eles e adivinhacao.
  //
  // Vale sobretudo para a queda de tensao: com Wi-Fi fraco o radio transmite em
  // potencia maxima e puxa picos de corrente, e fonte no limite reinicia o chip
  // sem deixar rastro. O que chega ao operador e "parou no meio do programa",
  // que nao se parece nada com problema de alimentacao.
  {
    const esp_reset_reason_t r = esp_reset_reason();
    const char* t = "desconhecido";
    switch (r) {
      case ESP_RST_POWERON:   t = "energia ligada (normal)";              break;
      case ESP_RST_SW:        t = "reinicio por software (OTA)";          break;
      case ESP_RST_PANIC:     t = "EXCECAO DE SOFTWARE (panic)";          break;
      case ESP_RST_INT_WDT:   t = "WATCHDOG DE INTERRUPCAO";              break;
      case ESP_RST_TASK_WDT:  t = "WATCHDOG DE TAREFA";                   break;
      case ESP_RST_WDT:       t = "WATCHDOG (outro)";                     break;
      case ESP_RST_BROWNOUT:  t = "QUEDA DE TENSAO (alimentacao fraca)";  break;
      case ESP_RST_DEEPSLEEP: t = "saida de sono profundo";               break;
      case ESP_RST_EXT:       t = "reset externo";                        break;
      default: break;
    }
    Serial.printf("MOTIVO DO ULTIMO REINICIO: %s (codigo %d)\n", t, (int)r);
    gMotivoDoReinicio = String(t) + " (codigo " + String((int)r) + ")";
  }

  gStatusMutex = xSemaphoreCreateMutex();
  gLogMutex = xSemaphoreCreateMutex();
  gMotorMutex = xSemaphoreCreateMutex();
  gConfigMutex = xSemaphoreCreateMutex();
  gLogQueue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(LogMessage));
  gDncCommandQueue = xQueueCreate(12, sizeof(DncCommand));

  if (!gStatusMutex || !gLogMutex || !gMotorMutex || !gConfigMutex
      || !gLogQueue || !gDncCommandQueue) {
    Serial.println("ERRO FATAL: memoria insuficiente para filas/semaforos");
    while (true) delay(1000);
  }

  setupWatchdogSystem();
  setupWatchdogForCurrentTask();

  if (!txRing.begin(TX_RING_SIZE) || !rxRing.begin(RX_RING_SIZE)) {
    Serial.println("ERRO: falha ao alocar buffers circulares");
    while (true) delay(1000);
  }

  settings.begin();
  gSerialConfig = settings.loadSerial();
  gNetworkConfig = settings.loadNetwork();
  gAccessConfig = settings.loadAccess();

  // CONFIGURACAO INVALIDA NAO E APAGADA DO CARTAO.
  //
  // A versao anterior gravava o padrao por cima (settings.saveSerial), e isso e
  // irreversivel: bastaria um firmware futuro apertar uma faixa de validacao
  // para a configuracao ajustada do cliente sumir, em disco, sem ele saber --
  // e so apareceria quando a maquina alarmasse.
  //
  // Aqui a caixa OPERA com o padrao (nao da para alimentar maquina com valor
  // fora de faixa) mas MANTEM salvo o que estava la. O aviso volta a cada
  // partida ate alguem corrigir, que e o que leva a correcao em vez de
  // esconder o problema.
  String erroConfig;
  if (!validarConfigSerial(gSerialConfig, erroConfig)) {
    Serial.printf("AVISO: configuracao serial salva invalida (%s). Operando com o padrao; o valor salvo foi PRESERVADO.\n",
                  erroConfig.c_str());
    gConfigSerialInvalida = erroConfig;
    gSerialConfig = DncSerialConfig();
  }

  // AVISA ENQUANTO ESTIVER COM SENHA DE FABRICA.
  //
  // Guardar a senha na memoria so resolve se alguem trocar. Sem este aviso, o
  // equipamento sai da bancada seguro no papel e igual a todos os outros na
  // pratica -- e isso so aparece depois, quando ja esta no cliente.
  if (settings.usandoSenhasDeFabrica(gAccessConfig)) {
    Serial.println("AVISO: senhas de acesso ainda sao as de fabrica. Troque-as na aba Rede.");
  }

  // O CARTAO NAO E INICIADO AQUI, DE PROPOSITO.
  //
  // `storage.begin()` pode demorar dezenas de segundos quando o cartao esta
  // ausente ou mal encaixado, e o watchdog derruba o equipamento antes disso
  // terminar -- laco de reinicio, sem Wi-Fi, sem portal, sem nenhuma indicacao
  // do motivo. Numa caixa dentro do painel da maquina, isso e a maquina parada.
  //
  // Um cartao com defeito NAO PODE impedir o equipamento de ligar. Quem monta o
  // cartao e a TaskStorage, que ja tentava de 2 em 2 segundos por conta propria
  // -- esta chamada aqui nao acrescentava nada alem do risco.
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  gStatus.sdOk = false;
  xSemaphoreGive(gStatusMutex);
  Serial.println("INFO: microSD sera montado pela TaskStorage");

  // MARCADORES DE PARTIDA.
  //
  // Ate aqui, um travamento dentro do setup() so aparecia como "watchdog
  // derrubou a loopTask", sem dizer onde. Estes avisos vao direto pela serial,
  // sem passar pela fila de log (a TaskLogger ainda nem existe), entao a ULTIMA
  // linha impressa e sempre o passo que travou. Custam alguns bytes e evitam
  // adivinhacao em campo.
  Serial.println("PASSO: iniciando serial RS232");
  serialManager.begin(gSerialConfig);
  serialManager.disconnectLines();

  Serial.println("PASSO: iniciando rede");
  startNetwork();

  Serial.println("PASSO: iniciando atualizacao pelo ar");
  iniciarOta();

  Serial.println("PASSO: iniciando protocolo do aplicativo");
  iniciarProtocolo();

  Serial.println("PASSO: iniciando portal web");
  webPortal.begin();

  Serial.println("PASSO: criando tarefas");
  bool tarefasOk = true;
  // Nucleos vem de Config.h: no C5 (um nucleo so) tudo vai para o 0.
  tarefasOk &= xTaskCreatePinnedToCore(TaskLogger,    "TaskLogger",    4096, nullptr, 1, nullptr, NUCLEO_REDE) == pdPASS;
  tarefasOk &= xTaskCreatePinnedToCore(TaskSerial,    "TaskSerial",    4096, nullptr, 4, nullptr, NUCLEO_TEMPO_REAL) == pdPASS;
  tarefasOk &= xTaskCreatePinnedToCore(TaskDNC,       "TaskDNC",       6144, nullptr, 3, nullptr, NUCLEO_TEMPO_REAL) == pdPASS;
  tarefasOk &= xTaskCreatePinnedToCore(TaskStorage,   "TaskStorage",   4096, nullptr, 1, nullptr, NUCLEO_REDE) == pdPASS;
  tarefasOk &= xTaskCreatePinnedToCore(TaskNetwork,   "TaskNetwork",   4096, nullptr, 1, nullptr, NUCLEO_REDE) == pdPASS;
  tarefasOk &= xTaskCreatePinnedToCore(TaskWebServer, "TaskWebServer", 8192, nullptr, 2, nullptr, NUCLEO_REDE) == pdPASS;
  if (!tarefasOk) {
    serialManager.disconnectLines();
    Serial.println("ERRO FATAL: falha ao criar uma ou mais tarefas; RS232 desabilitado");
    while (true) {
      esp_task_wdt_reset();
      delay(1000);
    }
  }

  // O motivo do reinicio so entra no historico depois que a fila de log existe.
  // QUEM TRAVOU, COM NUMERO.
  //
  // Depois de um watchdog, a tarefa com a marca de tempo mais ANTIGA e a que
  // parou de alimentar o cao -- as outras continuaram rodando ate o reset. A
  // diferenca para a mais recente diz por quanto tempo ela ficou presa.
  if (gMotivoDoReinicio.indexOf("WATCHDOG") >= 0) {
    if (gTinhaMarcasValidas) {
      uint32_t maisNova = 0;
      for (uint8_t i = 0; i < TAREFA_MAX; i++)
        if (gBatidaNoBootAnterior[i] > maisNova) maisNova = gBatidaNoBootAnterior[i];

      uint8_t culpada = 0xFF; uint32_t maisVelha = 0xFFFFFFFFu;
      for (uint8_t i = 0; i < TAREFA_MAX; i++) {
        if (gNomeNoBootAnterior[i][0] == 0) continue;      // tarefa nunca rodou
        if (gBatidaNoBootAnterior[i] < maisVelha) {
          maisVelha = gBatidaNoBootAnterior[i]; culpada = i;
        }
      }
      if (culpada != 0xFF) {
        logEventf("WARN", "Tarefa que travou: %s (parada ha %lu ms quando o cao mordeu)",
                  gNomeNoBootAnterior[culpada],
                  (unsigned long)(maisNova - maisVelha));
        for (uint8_t i = 0; i < TAREFA_MAX; i++) {
          if (gNomeNoBootAnterior[i][0] == 0) continue;
          logEventf("INFO", "  %s: ultima batida %lu ms (atraso %lu ms)",
                    gNomeNoBootAnterior[i],
                    (unsigned long)gBatidaNoBootAnterior[i],
                    (unsigned long)(maisNova - gBatidaNoBootAnterior[i]));
        }
      }
    } else {
      logEvent("INFO", "Sem marcas do boot anterior (primeira partida apos gravar).");
    }
  }


  if (gConfigSerialInvalida.length()) {
    logEventf("ERRO", "Configuracao serial salva e INVALIDA (%s). Operando com o "
                      "padrao; o valor salvo foi preservado. Corrija em Config.",
              gConfigSerialInvalida.c_str());
  }

  logEventf(gMotivoDoReinicio.indexOf("normal") >= 0
                || gMotivoDoReinicio.indexOf("OTA") >= 0 ? "INFO" : "WARN",
            "Motivo do ultimo reinicio: %s", gMotivoDoReinicio.c_str());
  logEvent("OK", "Sistema inicializado");
  setState(DNC_IDLE);
}

void loop() {
  esp_task_wdt_reset();
  updateStatusFlags();
  vTaskDelay(pdMS_TO_TICKS(1000));
}
