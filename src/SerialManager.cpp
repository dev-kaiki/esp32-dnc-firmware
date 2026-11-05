#include "SerialManager.h"
#include "../Config.h"

#include <algorithm>

// uart_set_sw_flow_ctrl() e do driver do IDF: o HardwareSerial do Arduino
// expoe o freio de CTS (setHwFlowCtrlMode) mas nao o de XON/XOFF.
#include <driver/uart.h>

// Definida no sketch: leva ao historico qual freio ficou valendo.
extern void smiLogFreio(bool noHardware, int codigo);

// Para apagar um unico bit que o driver liga por conta propria -- ver a nota
// sobre xonoff_del em applyConfig().

// Numero da UART vem da placa -- ver DNC_UART_NUM em Config.h. Um so lugar,
// para o HardwareSerial e o freio do IDF nunca apontarem para UARTs diferentes.
HardwareSerial DncSerial(DNC_UART_NUM);
static constexpr uart_port_t kUartDnc = static_cast<uart_port_t>(DNC_UART_NUM);

bool SerialManager::mesmaConfig(const DncSerialConfig& a, const DncSerialConfig& b) {
  // `timeoutMs` fica de fora de proposito: ele nao muda um bit no fio, e
  // reabrir a porta por causa dele seria pagar o glitch por nada.
  return a.baudRate == b.baudRate
      && a.dataBits == b.dataBits
      && a.parity   == b.parity
      && a.stopBits == b.stopBits
      && a.flow     == b.flow;
}

uint32_t SerialManager::serialConfigConstant(const DncSerialConfig& cfg) {
  // Mapeia configuracao flexivel para constantes aceitas pelo core Arduino-ESP32.
  // Nem todo CNC usa 5/6 bits; Fanuc classico costuma usar 7E1 ou 7E2.
  if (cfg.dataBits == 5) {
    if (cfg.parity == PARITY_EVEN && cfg.stopBits == 2) return SERIAL_5E2;
    if (cfg.parity == PARITY_EVEN) return SERIAL_5E1;
    if (cfg.parity == PARITY_ODD  && cfg.stopBits == 2) return SERIAL_5O2;
    if (cfg.parity == PARITY_ODD) return SERIAL_5O1;
    return cfg.stopBits == 2 ? SERIAL_5N2 : SERIAL_5N1;
  }
  if (cfg.dataBits == 6) {
    if (cfg.parity == PARITY_EVEN && cfg.stopBits == 2) return SERIAL_6E2;
    if (cfg.parity == PARITY_EVEN) return SERIAL_6E1;
    if (cfg.parity == PARITY_ODD  && cfg.stopBits == 2) return SERIAL_6O2;
    if (cfg.parity == PARITY_ODD) return SERIAL_6O1;
    return cfg.stopBits == 2 ? SERIAL_6N2 : SERIAL_6N1;
  }
  if (cfg.dataBits == 7) {
    if (cfg.parity == PARITY_EVEN && cfg.stopBits == 2) return SERIAL_7E2;
    if (cfg.parity == PARITY_EVEN) return SERIAL_7E1;
    if (cfg.parity == PARITY_ODD  && cfg.stopBits == 2) return SERIAL_7O2;
    if (cfg.parity == PARITY_ODD) return SERIAL_7O1;
    return cfg.stopBits == 2 ? SERIAL_7N2 : SERIAL_7N1;
  }
  // default 8 bits
  if (cfg.parity == PARITY_EVEN && cfg.stopBits == 2) return SERIAL_8E2;
  if (cfg.parity == PARITY_EVEN) return SERIAL_8E1;
  if (cfg.parity == PARITY_ODD  && cfg.stopBits == 2) return SERIAL_8O2;
  if (cfg.parity == PARITY_ODD) return SERIAL_8O1;
  return cfg.stopBits == 2 ? SERIAL_8N2 : SERIAL_8N1;
}

bool SerialManager::begin(const DncSerialConfig& cfg) {
  pinMode(PIN_RS232_RTS, OUTPUT);
  pinMode(PIN_RS232_DTR, OUTPUT);
  pinMode(PIN_RS232_CTS, INPUT_PULLUP);
  pinMode(PIN_RS232_DSR, INPUT_PULLUP);
  disconnectLines();
  return applyConfig(cfg);
}


// O FREIO DE FLUXO PRECISA SER ARMADO EM TODO CAMINHO. ESTE FOI O DEFEITO.
//
// `uart_set_sw_flow_ctrl` morava depois do atalho de "configuracao igual", la em
// cima. Como o aplicativo manda SEMPRE a mesma configuracao, o atalho disparava
// em todo envio e o freio NUNCA era armado. O periferico seguia deixando passar
// 80 bytes depois do DC3 -- contra os 10 que a Fanuc tolera -- e a Polaris
// alarmava 087 logo no inicio, medido em campo.
//
// E a SEGUNDA vez que este atalho engole uma inicializacao (antes foi o
// connectLines, ver a nota la em cima). Por isso agora e um metodo proprio,
// chamado dos DOIS caminhos, em vez de um trecho solto no meio da funcao.
void SerialManager::travarPorta() {
  if (_portaMutex) xSemaphoreTakeRecursive(_portaMutex, portMAX_DELAY);
}
void SerialManager::destravarPorta() {
  if (_portaMutex) xSemaphoreGiveRecursive(_portaMutex);
}

void SerialManager::configurarFreioDeFluxo(const DncSerialConfig& cfg) {
  if (cfg.flow == FLOW_XONXOFF) {
    const esp_err_t r = uart_set_sw_flow_ctrl(kUartDnc, true,
                                              UART_XONXOFF_LIMIAR_XON,
                                              UART_XONXOFF_LIMIAR_XOFF);
    _freioXonXoffNoHardware = (r == ESP_OK);

    // DIZER EM VOZ ALTA QUAL FREIO ESTA VALENDO.
    //
    // Eu implementei o freio no periferico e NAO expus isto em lugar nenhum --
    // nem log, nem status. Resultado: diante de um alarme 087 no cliente, nao
    // dava para saber se a correcao estava valendo ou se tinha falhado calada.
    // Uma correcao que nao da para verificar nao serve para campo.
    // No historico, nao so na serial: caixa em maquina nao tem cabo.
    smiLogFreio(_freioXonXoffNoHardware, (int)r);

    // NAO MEXER NO xonoff_del. MEDIDO NA POLARIS.
    //
    // Eu desligava este bit aqui para o firmware continuar ENXERGANDO o XOFF no
    // buffer de recepcao -- queria manter o log "XOFF recebido" e o estado
    // PAUSED_FLOW na tela.
    //
    // O QUE ISSO CUSTOU, com numero: na Polaris o freio deixou passar 80 BYTES
    // depois do DC3 (fila de saida 108 -> 28 em 250 ms). A Fanuc tolera 10, e
    // alarmou 087 em todo envio. Ou seja: eu tinha trocado o freio pela
    // visibilidade sem perceber.
    //
    // O IDF liga `sw_flow_con_en` e `xonoff_del` JUNTOS, e e assim que o
    // periferico trava o transmissor ao reconhecer o caractere. Desligar um
    // deles desarma o mecanismo.
    //
    // A visibilidade se resolve de outro jeito -- pela fila que para de drenar,
    // que a medida acima ja registra. O freio nao tem substituto.
  } else {
    uart_set_sw_flow_ctrl(kUartDnc, false, 0, 0);
    _freioXonXoffNoHardware = false;
  }
}

// DESCARTA O QUE FICOU PRESO NA SAIDA. MEDIDO NA POLARIS.
//
// `txRing.clear()` no cancelamento limpa a NOSSA fila, mas nao o buffer do
// driver da UART. Com a maquina segurando o XOFF, esses bytes nao saem -- ficam
// guardados esperando o XON.
//
// O QUE ISSO CAUSAVA: a cada envio cancelado sobrava mais entulho. O log mostrou
// a fila indo de 108 para 328 e depois 412 bytes, num arquivo de 316 -- ou seja,
// mais lixo do que programa. No envio seguinte esse entulho sai PRIMEIRO, e a
// maquina recebe o fim de um programa antigo colado na frente do novo.
//
// Nao existe API para esvaziar o buffer de saida (uart_flush so trata a
// entrada), entao o jeito e reabrir a porta. Isso gera um glitch no fio -- o
// mesmo que a nota em applyConfig descreve -- e por isso so acontece no
// CANCELAMENTO, quando a maquina ja nao esta lendo. E melhor um glitch com a
// maquina parada do que entulho no inicio do proximo programa.
size_t SerialManager::descartarSaidaPendente() {
  // ESTE E O PONTO DA CORRIDA.
  //
  // Quem cancela (TaskWebServer, pelo aplicativo ou pelo portal) chama isto, e
  // aqui dentro a porta e FECHADA e reaberta. A TaskSerial pode estar no meio
  // de um write() nesse instante. A trava faz uma esperar a outra.
  travarPorta();
  struct Solta { SerialManager* m; ~Solta(){ m->destravarPorta(); } } solta{this};
  if (!_iniciada) return 0;
  const size_t presos = bytesNaSaida();
  if (presos == 0) return 0;

  DncSerial.end();
  pinMode(PIN_RS232_TX, OUTPUT);
  digitalWrite(PIN_RS232_TX, HIGH);
  delay(5);

  DncSerial.setTxBufferSize(UART_TX_BUFFER);
  DncSerial.setRxBufferSize(UART_RX_BUFFER);
  DncSerial.begin(_cfg.baudRate, serialConfigConstant(_cfg),
                  PIN_RS232_RX, PIN_RS232_TX, false, _cfg.timeoutMs);
  DncSerial.setTimeout(_cfg.timeoutMs);

  configurarFreioDeFluxo(_cfg);
  connectLines();
  return presos;
}

bool SerialManager::applyConfig(const DncSerialConfig& cfg) {
  // A trava da porta nasce aqui, antes de qualquer uso. Enquanto nao existe,
  // travarPorta() nao faz nada -- e nao precisa: so ha uma tarefa viva.
  if (!_portaMutex) _portaMutex = xSemaphoreCreateRecursiveMutex();
  travarPorta();
  struct Solta { SerialManager* m; ~Solta(){ m->destravarPorta(); } } solta{this};
  // RECONFIGURAR A UART SUJA A LINHA. NAO FACA ISSO A TOA.
  //
  // O aplicativo manda CONFIG_SET antes de TODA operacao, quase sempre com a
  // mesma configuracao de antes. Reabrir a porta a cada vez custa um glitch no
  // fio -- e o glitch chega na maquina como um caractere quebrado.
  //
  // Custou uma Fanuc: ela alarmava COMMUNICATION ERROR (085) logo no inicio do
  // envio, sem ter recebido quase nada, enquanto a recepcao pela mesma porta
  // funcionava. A assimetria e o que denuncia: quando a maquina TRANSMITE ela
  // nao esta lendo a nossa linha, entao o glitch passa despercebido; quando ela
  // esta LENDO, ja em modo de recepcao, o glitch e o primeiro caractere que ela
  // ve -- framing error antes do programa comecar. A Mach9 tolera; a Fanuc nao.
  //
  // Configuracao igual, entao, nao mexe em nada.
  if (_iniciada && mesmaConfig(cfg, _cfg)) {
    // Sair cedo NAO pode pular isto. `disconnectLines()` roda no boot e no
    // CMD_DISCONNECT e deixa _connected em false; o laco de envio so escreve
    // com isConnected(). Sem reabilitar aqui, a caixa ficaria muda em TODA
    // maquina -- um estrago bem maior que o glitch que este trecho evita.
    // Sair cedo tambem NAO pode pular o freio: ver a nota no metodo.
    configurarFreioDeFluxo(cfg);
    connectLines();
    return true;
  }

  _cfg = cfg;

  // SEGURA O TX EM REPOUSO DURANTE A TROCA.
  //
  // Com a UART desligada o pino fica solto, e entrada solta no transceptor faz
  // a linha cair para espaco -- que a maquina le como start bit. Mantendo o
  // pino em nivel de repouso (alto na logica TTL), a linha continua parada e
  // nenhum caractere fantasma nasce.
  DncSerial.end();
  pinMode(PIN_RS232_TX, OUTPUT);
  digitalWrite(PIN_RS232_TX, HIGH);
  delay(30);

  // OS BUFFERS PRECISAM SER DEFINIDOS ANTES DO begin(), nao depois.
  //
  // Sem buffer de saida o `write()` fica preso ate o byte caber no FIFO do
  // hardware -- e tarefa presa nao enxerga o XOFF que a maquina acabou de
  // mandar. Ver a nota em UART_TX_BUFFER, no Config.h.
  DncSerial.setTxBufferSize(UART_TX_BUFFER);
  DncSerial.setRxBufferSize(UART_RX_BUFFER);

  DncSerial.begin(cfg.baudRate, serialConfigConstant(cfg), PIN_RS232_RX, PIN_RS232_TX, false, cfg.timeoutMs);
  DncSerial.setTimeout(cfg.timeoutMs);

  // FREIO DE CTS NO HARDWARE DA UART, NAO NO NOSSO LACO.
  //
  // POR QUE ISTO E NECESSARIO, MEDIDO NA SIEMENS CARREGANDO NCK:
  //
  // Reagir ao CTS por software significa olhar o pino de tempos em tempos e
  // parar de ALIMENTAR a fila. Mas o que ja esta na fila SAI DE QUALQUER JEITO
  // -- ate 64 caracteres, que a 9600 sao 66 ms de fio depois do \"pare\".
  //
  // A maquina baixou o CTS com pouca folga no buffer dela e transbordou nesses
  // 64. O mesmo arquivo passa pelo conversor USB porque o FTDI faz este freio
  // DENTRO DO CHIP: ele corta no meio do caractere seguinte, nao 64 depois.
  //
  // O ESP32 tem o mesmo recurso e nos nao estavamos usando. Com
  // UART_HW_FLOWCTRL_CTS o proprio periferico para a transmissao enquanto o CTS
  // estiver inativo -- reacao de um caractere, igual ao conversor.
  //
  // So no modo RTS/CTS: nos outros o pino pode nem estar ligado, e entregar o
  // controle da transmissao a um pino solto travaria o envio para sempre.
  if (cfg.flow == FLOW_RTSCTS) {
    DncSerial.setPins(PIN_RS232_RX, PIN_RS232_TX, PIN_RS232_CTS, -1);
    DncSerial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_CTS, 64);
  } else {
    DncSerial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_DISABLE, 64);
    DncSerial.setPins(PIN_RS232_RX, PIN_RS232_TX, -1, -1);
  }

  // O MESMO FREIO, AGORA PARA XON/XOFF. ESTE FALTAVA.
  //
  // MEDIDO NA POLARIS (Fanuc): ALARME 087 BUFFER OVERFLOW.
  //
  // O 087 nao e vago -- o manual da Fanuc diz o numero: depois de mandar DC3
  // ("pare"), a maquina aceita NO MAXIMO 10 CARACTERES. Passou disso, alarma.
  //
  // Reagir ao XOFF por software nao alcanca esse prazo, e nao e questao de
  // afinar constante nenhuma:
  //
  //   - a marca dagua do motor tem piso de 48 bytes (DncMotorCore), ja quase
  //     cinco vezes o que a Fanuc tolera;
  //   - e mesmo zerando isso, `bytesNaSaida()` mede o buffer do DRIVER e NAO
  //     enxerga o FIFO de 128 bytes do proprio periferico. O que ja desceu para
  //     o FIFO esta fora do alcance do nosso codigo: sai no fio aconteca o que
  //     acontecer.
  //
  // Ou seja: por software o prazo de 10 caracteres e inalcancavel por
  // construcao. Encolher a janela so empurraria o erro para mais adiante no
  // arquivo -- e ainda custaria vazao em baud alto, porque adiantar pouco
  // significa depender do nosso laco acordar a cada poucos milissegundos.
  //
  // O periferico do ESP32 sabe fazer isto sozinho: com sw_flow_con_en ele
  // reconhece o XOFF no fluxo de ENTRADA e trava o transmissor na hora, do
  // mesmo jeito que UART_HW_FLOWCTRL_CTS trava pelo pino. E o que o conversor
  // FTDI faz dentro do chip, e e por isso que o mesmo arquivo sempre passou
  // pelo cabo.
  //
  // POR QUE A MACH9 PASSOU E A POLARIS NAO: programa curto nao enche o buffer
  // da maquina, entao ela nunca chega a mandar DC3 e a janela nunca e posta a
  // prova. O defeito estava la o tempo todo, esperando um programa grande.
  //
  // Os dois limiares abaixo sao do sentido inverso (quando NOS mandamos
  // XON/XOFF porque o nosso buffer de recepcao encheu). Ficam largos de
  // proposito: quem recebe muito aqui e a captacao de programa vinda da CNC, e
  // interromper a maquina no meio disso seria pior que o problema.
  configurarFreioDeFluxo(cfg);

  _iniciada = true;
  _connected = true;
  connectLines();
  return true;
}

void SerialManager::connectLines() {
  digitalWrite(PIN_RS232_RTS, FLOW_OUTPUT_ACTIVE_LEVEL);
  digitalWrite(PIN_RS232_DTR, FLOW_OUTPUT_ACTIVE_LEVEL);
  _connected = true;
}

void SerialManager::disconnectLines() {
  digitalWrite(PIN_RS232_RTS, FLOW_OUTPUT_INACTIVE_LEVEL);
  digitalWrite(PIN_RS232_DTR, FLOW_OUTPUT_INACTIVE_LEVEL);
  _connected = false;
}

bool SerialManager::isConnected() const {
  return _connected;
}

int SerialManager::readByte() {
  if (!_connected) return -1;
  if (DncSerial.available() <= 0) return -1;
  return DncSerial.read();
}

bool SerialManager::writeByte(uint8_t b) {
  if (!_connected) return false;
  return DncSerial.write(&b, 1) == 1;
}

size_t SerialManager::readBlock(uint8_t* out, size_t maxLen) {
  travarPorta();
  struct Solta { SerialManager* m; ~Solta(){ m->destravarPorta(); } } solta{this};
  if (!_connected || maxLen == 0) return 0;
  const int disponivel = DncSerial.available();
  if (disponivel <= 0) return 0;
  const size_t quanto = std::min(maxLen, static_cast<size_t>(disponivel));
  return DncSerial.read(out, quanto);
}

size_t SerialManager::espacoLivreDeEscrita() {
  travarPorta();
  struct Solta { SerialManager* m; ~Solta(){ m->destravarPorta(); } } solta{this};
  if (!_connected) return 0;
  const int livre = DncSerial.availableForWrite();
  return livre > 0 ? static_cast<size_t>(livre) : 0;
}

size_t SerialManager::bytesNaSaida() {
  if (!_connected) return 0;
  const size_t livre = espacoLivreDeEscrita();
  return livre >= UART_TX_BUFFER ? 0 : (UART_TX_BUFFER - livre);
}

size_t SerialManager::writeBlock(const uint8_t* dados, size_t n) {
  travarPorta();
  struct Solta { SerialManager* m; ~Solta(){ m->destravarPorta(); } } solta{this};
  if (!_connected || n == 0) return 0;

  // NUNCA ESCREVER MAIS DO QUE CABE.
  //
  // Esta e a linha que impede o bloqueio: `write()` so nao prende a tarefa
  // enquanto houver espaco no buffer de saida. Passar disso devolveria o
  // comportamento que esta correcao existe para eliminar.
  const size_t cabe = espacoLivreDeEscrita();
  if (cabe == 0) return 0;
  const size_t quanto = std::min(n, cabe);
  return DncSerial.write(dados, quanto);
}

bool SerialManager::enviarControleDeFluxo(uint8_t caractere) {
  travarPorta();
  struct Solta { SerialManager* m; ~Solta(){ m->destravarPorta(); } } solta{this};
  if (!_connected) return false;
  // Escrita direta, sem passar pela fila e sem consultar permissao de envio.
  // Ver a nota no cabecalho.
  return DncSerial.write(&caractere, 1) == 1;
}

void SerialManager::flush() {
  travarPorta();
  struct Solta { SerialManager* m; ~Solta(){ m->destravarPorta(); } } solta{this};
  DncSerial.flush();
}

bool SerialManager::ctsOk() {
  return digitalRead(PIN_RS232_CTS) == FLOW_INPUT_ACTIVE_LEVEL;
}

bool SerialManager::dsrOk() {
  return digitalRead(PIN_RS232_DSR) == FLOW_INPUT_ACTIVE_LEVEL;
}

bool SerialManager::canTransmit(const DncSerialConfig& cfg, bool xonAllowed) {
  if (!_connected) return false;
  if (cfg.flow == FLOW_XONXOFF) return xonAllowed;
  if (cfg.flow == FLOW_RTSCTS) return ctsOk();
  if (cfg.flow == FLOW_DTRDSR) return dsrOk();
  return true;
}
