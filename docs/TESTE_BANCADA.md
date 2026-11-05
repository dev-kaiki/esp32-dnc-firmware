# Roteiro de teste de bancada

## 1. Teste sem CNC

1. Grave o firmware no ESP32.
2. Coloque microSD FAT32.
3. Ligue o ESP32.
4. Conecte no AP `SMI_DNC_ESP32` senha `12345678`.
5. Acesse `http://192.168.4.1`.
6. Login: `admin` / `smiadmin`.
7. Faça upload de um arquivo `.nc` em `/programs`.
8. Verifique se aparece no gerenciador.

## 2. Teste de loopback TTL antes do MAX3232

Use apenas em bancada:

1. Desligue o MAX3232 da CNC.
2. Jumpe GPIO17 TX2 com GPIO16 RX2.
3. Configure fluxo `NONE`.
4. Clique em receber e depois enviar um arquivo pequeno.
5. Os bytes enviados devem voltar como recebidos.

## 3. Teste com dois conversores RS232

1. ESP32 + MAX3232 para DB9.
2. Outro conversor USB-RS232 no PC.
3. Configure os mesmos parâmetros no terminal serial do PC.
4. Teste envio e recepção.
5. Teste XON/XOFF enviando CTRL+S e CTRL+Q no terminal.

## 4. Teste com CNC real

Antes de conectar:

- Verifique pinagem DB9/DB25 da CNC.
- Verifique se a CNC usa cabo null-modem ou direto.
- Verifique baud, paridade, stop bits e ISO/EIA.
- Verifique se o controle usa DC1/DC3 para XON/XOFF.
- Faça backup dos parâmetros da CNC antes de qualquer alteração.

Sequência:

1. CNC em modo EDIT/INPUT para receber arquivo.
2. DNC: Enviar para CNC.
3. CNC em modo PUNCH/OUTPUT para enviar arquivo.
4. DNC: Receber da CNC.
5. CNC em modo TAPE/DNC/REMOTE para drip-feed.
6. DNC: Executar Online.

## 5. O que observar

- O log precisa mostrar XOFF/XON quando a CNC pausar/retomar.
- A barra de progresso precisa avançar.
- O contador de bytes precisa aumentar.
- CTS/DSR precisam mudar conforme o hardware real.
- Se der timeout final, o CNC parou de aceitar dados ou há bloqueio de fluxo.

## 6. Erros comuns

| Sintoma | Causa provável |
|---|---|
| CNC não recebe nada | TX/RX invertido, cabo errado, baud/paridade errada |
| Caracteres estranhos | Baud/paridade/data bits incorretos |
| Envio para no meio | XOFF sem XON, CTS bloqueado, buffer CNC cheio |
| Recepção vazia | CNC não está em PUNCH/OUTPUT, RX errado |
| Drip-feed falha | CNC não está em modo TAPE/DNC/REMOTE |
| Reinício do ESP32 | Fonte fraca, ruído, GND/isolação ruim |
