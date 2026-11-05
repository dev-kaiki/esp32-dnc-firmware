# SMI ESP32 DNC Industrial

Firmware base para um equipamento DNC industrial usando **ESP32 DevKit V1 30 pinos**, **MAX3232**, **microSD** e **interface web embarcada**.

> Estado do projeto: base funcional/profissional para bancada, validação com CNC real e evolução de produto. Antes de produção, validar eletricamente RS232, aterramento, isolação, conectores, pinagem DB9/DB25 e protocolo de cada controle CNC.

---

## 1. Objetivo

O equipamento opera como um DNC para CNCs compatíveis com RS232:

- Armazena programas CNC no microSD.
- Envia arquivos para memória da CNC.
- Recebe arquivos da CNC.
- Executa programa em modo online / drip-feed.
- Controla fluxo por XON/XOFF, RTS/CTS, DTR/DSR ou sem controle.
- Usa servidor web local para operação, configuração e arquivos.
- Usa FreeRTOS com tarefas independentes.

---

## 2. Arquitetura geral

```text
+------------------------+        +------------------------+
| Navegador / Operador   | <----> | ESP32 WebServer        |
+------------------------+        +-----------+------------+
                                             |
                                             v
+------------------------+        +-----------+------------+
| microSD                | <----> | TaskDNC / TaskStorage  |
| /programs              |        +-----------+------------+
| /received              |                    |
| /logs                  |                    v
+------------------------+        +-----------+------------+
                                  | Buffer Circular TX/RX  |
                                  +-----------+------------+
                                              |
                                              v
                                  +-----------+------------+
                                  | TaskSerial / UART2     |
                                  +-----------+------------+
                                              |
                                              v
                                  +-----------+------------+
                                  | MAX3232 / RS232        |
                                  +-----------+------------+
                                              |
                                              v
                                  +-----------+------------+
                                  | CNC Fanuc/Siemens/etc  |
                                  +------------------------+
```

---

## 3. Diagrama de blocos físico

```text
Fonte 24V industrial
        |
        +--> Conversor 24V -> 5V ----> ESP32 VIN/5V
        |
        +--> Conversor 24V -> 3.3V opcional, se a placa exigir alimentação dedicada

ESP32 DevKit V1
  GPIO17 TX2 ------------------> MAX3232 T1IN -> RS232 TX -> CNC RX
  GPIO16 RX2 <------------------ MAX3232 R1OUT <- RS232 RX <- CNC TX
  GPIO25 RTS ------------------> MAX3232 canal extra -> CNC CTS
  GPIO26 CTS <------------------ MAX3232 canal extra <- CNC RTS/CTS
  GPIO27 DTR ------------------> MAX3232 canal extra -> CNC DSR
  GPIO33 DSR <------------------ MAX3232 canal extra <- CNC DTR/DSR

microSD SPI
  CS   -> GPIO5
  SCK  -> GPIO18
  MISO -> GPIO19
  MOSI -> GPIO23
  VCC  -> 3.3V preferencialmente
  GND  -> GND comum
```

### Observação crítica sobre MAX3232

Um módulo MAX3232 simples normalmente tem 2 drivers e 2 receivers. Isso atende:

- TX/RX básicos; ou
- TX/RX + um par parcial de handshake, dependendo da placa.

Para **TX/RX + RTS/CTS + DTR/DSR completos**, use conversor RS232 com canais suficientes ou mais de um CI/módulo MAX3232.

---

## 4. Estrutura de diretórios

```text
SMI_ESP32_DNC/
  SMI_ESP32_DNC.ino
  Config.h
  README.md
  docs/
    FLUXOGRAMAS.md
    TESTE_BANCADA.md
  src/
    Types.h
    CircularBuffer.h
    CircularBuffer.cpp
    Logger.h
    Logger.cpp
    Settings.h
    Settings.cpp
    SerialManager.h
    SerialManager.cpp
    StorageManager.h
    StorageManager.cpp
    WebPortal.h
    WebPortal.cpp
```

---

## 5. Tarefas FreeRTOS

| Tarefa | Função | Prioridade |
|---|---:|---:|
| TaskSerial | Leitura/escrita UART RS232 e controle de fluxo | Alta |
| TaskDNC | Envio, recepção, drip-feed, pausa, retomada, cancelamento | Alta/média |
| TaskStorage | Diagnóstico e recuperação do microSD | Baixa |
| TaskNetwork | Reconexão Wi-Fi | Baixa |
| TaskWebServer | Interface web embarcada | Média |
| TaskLogger | Logs em RAM e microSD | Baixa |

---

## 6. Modos de operação

### 6.1 Enviar para CNC

```text
Arquivo no SD -> leitura em blocos -> buffer TX -> UART -> CNC
```

Não carrega o arquivo inteiro na RAM.

### 6.2 Receber da CNC

```text
CNC -> UART -> buffer RX -> escrita em arquivo no SD
```

Finaliza automaticamente por silêncio após `timeoutMs` quando já recebeu algum dado.

### 6.3 Drip-feed / DNC Online

```text
microSD -> buffer circular -> RS232 -> CNC executando em tempo real
```

A transmissão para quando:

- Recebe XOFF;
- CTS bloqueia;
- DSR bloqueia;
- operador pausa;
- operador cancela.

A transmissão retoma quando:

- Recebe XON;
- CTS libera;
- DSR libera;
- operador retoma.

---

## 7. Interface Web

Acesse pelo navegador:

- Se Wi-Fi configurado: `http://IP_DO_ESP32`
- Se não houver Wi-Fi configurado: conecte no AP `SMI_DNC_ESP32`, senha `12345678`, e acesse `http://192.168.4.1`

Usuários iniciais:

| Nível | Usuário | Senha |
|---|---|---|
| Administrador | admin | smiadmin |
| Operador | operador | operador |
| Visualização | viewer | viewer |

> Trocar isso antes de qualquer uso real. A versão atual usa HTTP local sem TLS.

---

## 8. Configurações RS232 iniciais

Padrão inicial:

- Baud: 9600
- Data Bits: 7
- Paridade: EVEN
- Stop Bits: 1
- Fluxo: XON/XOFF
- Timeout: 3000 ms

Perfis comuns que você vai encontrar em CNCs antigos:

| Controle | Configuração comum |
|---|---|
| Fanuc | 4800/9600, 7E1 ou 7E2, XON/XOFF |
| Mitsubishi | 9600, 7E1/8N1, XON/XOFF |
| Siemens | varia muito; validar manual do controle |
| Fagor | 9600, 7E1 ou 8N1 |
| Heidenhain | depende do modo TNC/LSV2; validar antes |

---

## 9. Como compilar na Arduino IDE

1. Instale o core **esp32 by Espressif Systems** no Gerenciador de Placas.
2. Selecione uma placa compatível, por exemplo: **ESP32 Dev Module**.
3. Abra `SMI_ESP32_DNC.ino`.
4. Confira os pinos em `Config.h`.
5. Compile e grave.
6. Formate o microSD em FAT32.
7. Insira o microSD e reinicie o ESP32.
8. Conecte no Wi-Fi/AP e acesse a interface web.

Bibliotecas usadas: apenas bibliotecas comuns do core Arduino-ESP32 e Arduino:

- WiFi
- WebServer
- SD
- SPI
- Preferences
- HardwareSerial
- FreeRTOS nativo do ESP32

---

## 10. Limitações intencionais desta base

- HTTPS não está implementado por padrão; em chão de fábrica isolado pode ser aceitável, mas para rede corporativa precisa TLS/reverse proxy ou gateway.
- Login inicial é simples e deve ser substituído por senha persistente editável.
- Ethernet e USB pendrive estão preparados conceitualmente, mas não implementados no firmware atual.
- DTR/DSR depende de hardware RS232 com canais suficientes.
- O drip-feed precisa ser ajustado no CNC real, principalmente caracteres de início/fim, EOB, eco, timeout e modo de recepção.

---

## 11. Próximas evoluções recomendadas

1. Criar placa dedicada com isolação RS232 industrial.
2. Adicionar perfis salvos por máquina.
3. Adicionar tela TFT local.
4. Implementar Ethernet W5500 ou LAN8720.
5. Implementar USB Host para pendrive.
6. Adicionar checksum/CRC em backup.
7. Adicionar servidor central DNC com sincronização.
8. Adicionar atualização OTA assinada.
9. Adicionar usuários editáveis e troca de senha.
10. Adicionar banco local leve para histórico avançado.
