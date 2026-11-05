# Fluxogramas - SMI ESP32 DNC

## 1. Inicialização

```text
INICIO
  |
  v
Inicializa Serial Debug
  |
  v
Cria mutex, filas e buffers circulares
  |
  v
Carrega configuracoes NVS/Preferences
  |
  v
Inicializa microSD
  |
  v
Inicializa UART RS232
  |
  v
Inicializa Wi-Fi ou AP local
  |
  v
Inicializa WebServer
  |
  v
Cria tarefas FreeRTOS
  |
  v
Sistema parado aguardando comando
```

## 2. Envio para CNC

```text
Operador seleciona arquivo
  |
  v
WebServer envia CMD_SEND_FILE
  |
  v
TaskDNC abre arquivo no SD
  |
  v
Le bloco de 256 bytes
  |
  v
Ha espaco no buffer TX?
  |-- nao --> aguarda
  |-- sim --> grava bloco no buffer
  |
  v
TaskSerial verifica fluxo
  |
  +-- XOFF? pausa
  +-- CTS bloqueado? pausa
  +-- DSR bloqueado? pausa
  +-- usuario pausou? pausa
  |
  v
Envia byte pela UART
  |
  v
Fim do arquivo?
  |-- nao --> proximo bloco
  |-- sim --> aguarda drenar buffer
  |
  v
Concluido
```

## 3. Drip-feed

```text
CNC em modo DNC/ONLINE
  |
  v
Operador clica Executar Online
  |
  v
TaskDNC le arquivo por blocos
  |
  v
Buffer circular alimenta UART continuamente
  |
  v
CNC solicita pausa?
  |-- XOFF/CTS/DSR --> pausa imediata
  |-- liberou --> retoma automaticamente
  |
  v
Arquivo pode ser maior que a RAM
  |
  v
Fim do arquivo e buffer drenado
  |
  v
Execucao finalizada
```

## 4. Recepção

```text
Operador clica Receber
  |
  v
TaskDNC abre arquivo em /received
  |
  v
TaskSerial recebe bytes da CNC
  |
  v
Bytes entram no buffer RX
  |
  v
TaskDNC grava blocos no SD
  |
  v
Silencio maior que timeout depois de receber dados?
  |-- nao --> continua
  |-- sim --> fecha arquivo
  |
  v
Recepcao concluida
```

## 5. Controle de fluxo XON/XOFF

```text
TaskSerial recebe byte da CNC
  |
  +-- 0x13 XOFF --> gXonAllowed = false --> pausa TX
  |
  +-- 0x11 XON  --> gXonAllowed = true  --> retoma TX
  |
  +-- outro     --> grava no buffer RX
```

## 6. Cancelamento

```text
Operador clica Cancelar
  |
  v
WebServer envia CMD_CANCEL
  |
  v
TaskDNC marca cancelRequested
  |
  v
Limpa buffer TX
  |
  v
Fecha arquivo aberto
  |
  v
Volta para conectado/parado
```
