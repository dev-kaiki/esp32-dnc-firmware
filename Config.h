#pragma once

#include <sdkconfig.h>

/*
  SMI ESP32 DNC Industrial - Configuracao de hardware

  DUAS PLACAS, UM CODIGO SO. O chip escolhido na compilacao decide qual pinagem
  vale -- assim o prototipo (que roda nas maquinas hoje) e a placa de producao
  nunca divergem em regra de negocio, so em numero de pino.

    ESP32 classico  -> prototipo: ESP32 DevKit V1 30 pinos em protoboard
    ESP32-C5        -> placa de producao "DNC ESP32 C5 WROOM 1U" (Rev Lukita,
                       2026-10-05), modulo ESP32-C5-WROOM-1U-N8R8

  Prototipo:

  Ligacao principal:
    ESP32 GPIO17 TX2 -> MAX3232 T1IN -> RS232 TX para CNC RX
    ESP32 GPIO16 RX2 <- MAX3232 R1OUT <- RS232 RX vindo CNC TX

  Observacao importante:
    Para usar RTS/CTS + DTR/DSR completos, o conversor RS232 precisa ter canais suficientes.
    Um modulo MAX3232 simples costuma ter 2 drivers e 2 receivers. TX/RX + RTS/CTS consome
    2 drivers e 2 receivers. DTR/DSR exige canais adicionais ou outro CI/modulo.
*/

#if CONFIG_IDF_TARGET_ESP32C5
// =========================== PLACA DE PRODUCAO (C5) ===========================
//
// Pinagem lida do esquematico (pagina 1). DB9 macho com pinagem de DTE, igual a
// de um PC: a caixa usa o mesmo cabo que o computador usaria.
#define PLACA_NOME          "SMI DNC C5 rev1"

// ---------- UART RS232 (U2 MAX3232: TX/RX + RTS/CTS; U3 MAX3232: DTR/DSR) ----
#define PIN_RS232_RX        24   // U2 R1OUT <- DB9-2
#define PIN_RS232_TX        23   // U2 T1IN  -> DB9-3
#define PIN_RS232_RTS        5   // U2 T2IN  -> DB9-7
#define PIN_RS232_CTS        4   // U2 R2OUT <- DB9-8
#define PIN_RS232_DTR        0   // U3 T1IN  -> DB9-4
#define PIN_RS232_DSR        1   // U3 R1OUT <- DB9-6

// UART 1, NAO 2.
//
// No C5 a "UART 2" e a LP UART: pinos fixos (4 e 5) e FIFO de 16 bytes. O
// HardwareSerial(2) compila, mas recusa os pinos do RS232 em execucao -- e os
// limiares do freio XON/XOFF (32/100) nao cabem em 16 bytes, entao o freio
// por hardware falharia calado. A UART 1 tem os mesmos 128 bytes do prototipo.
#define DNC_UART_NUM         1

// ---------- microSD e CH376T no MESMO barramento SPI ----------
//
// O C5 tem um SPI de uso geral so: cartao e CH376 dividem SCK/MOSI/MISO.
#define PIN_SD_CS           10
#define PIN_SD_SCK           6
#define PIN_SD_MISO          9
#define PIN_SD_MOSI          8

// CH376T (USB host para pendrive). SEM DRIVER AINDA -- mas o chip esta no
// barramento, entao o CS dele tem de ficar DESATIVADO (alto) o tempo todo.
//
// Na placa, CS e INT cairam nos pinos da UART 0 do chip (U0TXD/U0RXD), por
// onde o sistema escreve o log. Cada linha de log selecionaria o CH376 no
// meio de uma leitura do cartao, e a saida dele brigaria com a do cartao no
// MISO. Tomar o pino como GPIO comum tira a UART 0 dele -- ver
// prenderCh376ForaDoBarramento().
#define PIN_CH376_CS        11
#define PIN_CH376_INT       12

// Sinalizacao e botao da placa.
#define PIN_LED_ALARME       3   // LED AL, acende em nivel ALTO
#define PIN_BOTAO_BOOT      28   // SW2, ativo em nivel BAIXO (tem pull-up)

#else
// ============================= PROTOTIPO (ESP32) ==============================
#define PLACA_NOME          "SMI DNC prototipo ESP32"

// ---------- UART RS232 ----------
#define PIN_RS232_RX        16
#define PIN_RS232_TX        17
#define PIN_RS232_RTS       25   // Saida TTL para driver RS232: ESP32 RTS -> CNC CTS
#define PIN_RS232_CTS       26   // Entrada TTL de receiver RS232: CNC RTS/CTS -> ESP32 CTS
#define PIN_RS232_DTR       27   // Saida TTL para driver RS232: ESP32 DTR -> CNC DSR
#define PIN_RS232_DSR       33   // Entrada TTL de receiver RS232: CNC DTR/DSR -> ESP32 DSR
#define DNC_UART_NUM         2

// ---------- microSD SPI ----------
#define PIN_SD_CS           5
#define PIN_SD_SCK          18
#define PIN_SD_MISO         19
#define PIN_SD_MOSI         23
#endif

// Nucleos das tarefas.
//
// O ESP32 classico tem dois: a serial e o DNC ficam no 1, longe do Wi-Fi. O C5
// tem UM so -- e fixar tarefa no nucleo 1 la e assert no boot, a caixa reinicia
// em laco. Com um nucleo, quem protege a maquina do atraso do Wi-Fi e o freio
// XON/XOFF por hardware da UART, que nao depende de tarefa nenhuma.
// Escolha EXPLICITA por chip: uma macro de contagem de nucleos que nao existisse
// nesta versao do core cairia no #else calado e mudaria o prototipo.
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_FREERTOS_UNICORE
#define NUCLEO_TEMPO_REAL    0
#else
#define NUCLEO_TEMPO_REAL    1
#endif
#define NUCLEO_REDE          0

// Na maioria dos conversores MAX3232, sinais de controle RS232 assertados chegam como LOW no lado TTL.
#define FLOW_INPUT_ACTIVE_LEVEL LOW
#define FLOW_OUTPUT_ACTIVE_LEVEL LOW
#define FLOW_OUTPUT_INACTIVE_LEVEL HIGH

// Velocidade do SPI do cartao.
//
// 4 MHz, nao 25 MHz. Vinte e cinco megahertz e velocidade de trilha curta em
// placa impressa; em jumper de protoboard o sinal nao chega inteiro, o cartao
// nao responde, e a biblioteca fica insistindo por dezenas de segundos --
// tempo suficiente para o watchdog derrubar o equipamento no boot.
//
// Nao ha nada a ganhar com mais: 4 MHz dao centenas de KB/s, e a CNC mais
// rapida que atendemos consome menos de 10 KB/s.
#define SD_SPI_HZ           4000000

// ---------- Identificacao do firmware ----------
//
// PARA QUE SERVE: responder "esta caixa esta rodando a versao nova?" sem ir ate
// o cliente. Depois de uma atualizacao pelo ar, sao estes dois valores, junto
// com a particao em uso, que provam que a gravacao pegou.
//
// SUBA A VERSAO A CADA ENTREGA. Sem isso, duas versoes diferentes se apresentam
// com o mesmo numero e a conferencia nao vale nada.
//
// 1.2.0: caixa esquece os opIds vistos quando o app cai (app reaberto nao tem
// mais comando confirmado sem executar) e port para a placa de producao C5.
#define FIRMWARE_VERSION    "1.2.0"

// Carimbo do momento da compilacao. Distingue duas compilacoes da mesma versao
// -- util justamente em teste de OTA, quando se envia o mesmo codigo de novo.
#define FIRMWARE_BUILD      __DATE__ " " __TIME__

// ---------- Sistema ----------
#define HOSTNAME            "smi-esp32-dnc"
#define DEFAULT_AP_SSID     "SMI_DNC_ESP32"
#define DEFAULT_AP_PASS     "12345678"
#define HTTP_PORT           80

// Porta do protocolo binario com o aplicativo Android.
//
// 3333, NAO 3232: a 3232 e do ArduinoOTA, que roda neste mesmo equipamento. Duas
// coisas nao escutam na mesma porta -- o servidor de protocolo nao subiria, e a
// falha apareceria como "o aplicativo nao conecta na caixa", sem dizer por que.
#define DNC_PROTOCOL_PORT   3333

// ---------- Buffers ----------
#define TX_RING_SIZE        8192
#define RX_RING_SIZE        8192

// Buffers da UART do driver, EM BYTES.
//
// O DE SAIDA NAO PODE SER ZERO, e essa e a parte que importa.
//
// Sem buffer de saida o `write()` da HardwareSerial fica preso ate o byte caber
// no FIFO do hardware. Escrever 128 bytes a 9600 baud prenderia a tarefa por
// ~133 ms, e durante esse tempo ela nao ve um XOFF chegando -- a maquina pede
// para parar e continua recebendo. Com buffer, a escrita copia e volta na hora,
// e o `availableForWrite()` diz quanto cabe sem bloquear.
//
// 1024 bytes sao mais de um segundo de fio a 9600 baud: folga de sobra para o
// motor trabalhar, e pequeno o bastante para o "parar" do operador nao demorar
// um tempo perceptivel.
#define UART_TX_BUFFER      1024

// Quantos bytes o motor pode deixar esperando na fila da UART.
//
// NAO confundir com UART_TX_BUFFER: aquele e a folga do driver, este e o quanto
// nos nos adiantamos. E o mesmo numero do caminho por conversor USB, que envia
// para Fanuc sem alarmar. Ver a nota em traduzirConfigDoMotor.
#define JANELA_DE_ESCRITA   256

// Janela quando o controle de fluxo e XON/XOFF.
//
// Menor de proposito: e o numero de caracteres que a maquina ainda vai receber
// depois de pedir para parar. Ver a nota em traduzirConfigDoMotor.
#define JANELA_XONXOFF      64

// Limiares do XON/XOFF que NOS enviamos, em bytes do buffer de recepcao.
//
// Sentido inverso do freio: sao para quando o nosso buffer enche recebendo
// programa DA maquina. Largos de proposito -- quem enche isto e a captacao, e
// mandar a CNC parar no meio de uma transmissao dela seria pior que o aperto.
// OS DOIS PRECISAM SER MENORES QUE O FIFO DO PERIFERICO (128 no ESP32).
//
// Eu tinha posto 200 aqui, "largo de proposito". O IDF valida contra
// UART_HW_FIFO_LEN e REJEITA a chamada inteira:
//
//   ESP_RETURN_ON_FALSE(rx_thresh_xoff < UART_HW_FIFO_LEN(uart_num), ESP_FAIL, ...)
//
// Resultado medido na Polaris: uart_set_sw_flow_ctrl devolvia -1, o freio NUNCA
// era armado, e o periferico deixava passar 80 bytes depois do DC3 -- contra os
// 10 que a Fanuc tolera. Um numero fora de faixa desligou o freio inteiro, em
// silencio, porque ninguem olhava o retorno.
#define UART_XONXOFF_LIMIAR_XON    32
#define UART_XONXOFF_LIMIAR_XOFF  100
#define UART_RX_BUFFER      2048
#define SD_READ_BLOCK       256
#define SD_WRITE_BLOCK      256
#define LOG_QUEUE_LENGTH    40
// Historico em memoria. De 80 para 200: um envio que falha rende varias linhas
// (fluxo, medida do freio, estado, erro), e com 80 o comeco da falha -- a parte
// que interessa -- ja tinha sido descartado quando alguem ia olhar.
#define MAX_RECENT_LOGS     200

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
