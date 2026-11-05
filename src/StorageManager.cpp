#include "StorageManager.h"
#include <esp_task_wdt.h>
#include "Logger.h"
#include "../Config.h"

// ATENCAO: esta funcao PODE demorar. Nao chame de dentro do setup().
//
// Quando o cartao esta ausente, mal encaixado ou o barramento nao fecha, a
// biblioteca SD insiste por dezenas de segundos antes de desistir. Quem chama
// precisa estar preparado para isso -- ver a nota na TaskStorage sobre sair do
// watchdog durante a tentativa.
bool StorageManager::begin() {
  if (!_sdMutex) _sdMutex = xSemaphoreCreateMutex();

  // REMONTAR O CARTAO PRECISA DO MUTEX MAIS DO QUE QUALQUER OUTRA COISA.
  //
  // Aqui o driver e DESTRUIDO e RECRIADO (SD.end, SPI.begin, SD.begin). Sem o
  // mutex, isso acontecia por baixo de quem estivesse lendo ou gravando naquele
  // instante -- e a TaskStorage repete a tentativa a cada 5 segundos enquanto o
  // cartao nao responde, que e exatamente quando o equipamento esta mais
  // exposto.
  //
  // `_ok = false` ANTES de soltar o mutex: assim quem chegar depois desiste na
  // guarda de `_ok` em vez de falar com um driver meio montado.
  xSemaphoreTake(_sdMutex, portMAX_DELAY);

  if (_ok) SD.end();
  _ok = false;

#ifdef PIN_CH376_CS
  // O CH376 divide o barramento: desativado ANTES de o SPI acordar.
  pinMode(PIN_CH376_CS, OUTPUT);
  digitalWrite(PIN_CH376_CS, HIGH);
#endif
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  const bool montou = SD.begin(PIN_SD_CS, SPI, SD_SPI_HZ);
  _ok = montou;

  xSemaphoreGive(_sdMutex);

  // Fora do mutex de proposito: ensureBaseDirs() usa os metodos publicos da
  // classe, que pegam o mutex por conta. Chamar aqui dentro seria travar contra
  // nos mesmos -- o mutex do FreeRTOS nao e recursivo.
  if (montou) ensureBaseDirs();
  return montou;
}

bool StorageManager::isOk() const {
  return _ok;
}

SemaphoreHandle_t StorageManager::mutex() {
  return _sdMutex;
}

String StorageManager::sanitizePath(const String& input) {
  // SEGMENTO A SEGMENTO. NAO APAGUE ".." DE DENTRO DO NOME.
  //
  // A versao anterior fazia `p.replace("..", "")`, o que remove dois pontos
  // seguidos de QUALQUER lugar do caminho -- inclusive do meio de um nome de
  // arquivo legitimo. Um programa chamado
  // "DISPOSITIVOPARAGRAVARPLAQUETAS..NC" era GRAVADO com o nome inteiro e
  // depois PROCURADO como "...PLAQUETASNC". O envio morria em "Arquivo nao
  // encontrado no microSD", que na tela do operador vira so "Falha", e o
  // mesmo programa funcionava pelo conversor USB -- onde o cartao da caixa e
  // este saneador nem entram no caminho.
  //
  // O que a protecao precisa impedir e SUBIR DE PASTA, ou seja, um segmento
  // que SEJA "..". Um segmento que apenas CONTENHA pontos e um nome de
  // arquivo comum e tem que passar intacto. Montar o caminho segmento a
  // segmento resolve os dois lados de uma vez, e ainda colapsa "//" e "."
  // sem precisar de passes de troca de texto que se atrapalham entre si.
  String p = input;
  p.trim();
  p.replace("\\", "/");

  String saida;
  const int n = (int)p.length();
  int i = 0;
  while (i <= n) {
    int barra = p.indexOf('/', i);
    if (barra < 0) barra = n;
    const String seg = p.substring(i, barra);
    i = barra + 1;

    if (seg.length() == 0) continue;      // barra dupla ou inicio do caminho
    if (seg == "." || seg == "..") continue;  // nao anda para fora da pasta

    saida += "/";
    saida += seg;
  }

  if (saida.length() == 0) saida = "/";
  return saida;
}

String StorageManager::sanitizeFileName(const String& bruto) {
  String n = bruto;

  // 1. RECUPERA O NOME REAL. A aspa que fecha o nome e o limite verdadeiro;
  //    tudo depois dela e resto de cabecalho que o analisador deixou vazar.
  int corte = n.indexOf('"');
  if (corte >= 0) n = n.substring(0, corte);

  // 2. Rede de seguranca para o caso sem aspas.
  corte = n.indexOf(';');
  if (corte >= 0) n = n.substring(0, corte);

  n.trim();

  // 3. So o nome, nunca caminho. Sem isto, um nome com ".." ou com barra
  //    gravaria fora da pasta de programas.
  int barra = n.lastIndexOf('/');
  if (barra >= 0) n = n.substring(barra + 1);
  barra = n.lastIndexOf('\\');
  if (barra >= 0) n = n.substring(barra + 1);
  n.trim();

  // 4. Caracteres que o sistema de arquivos do cartao nao aceita, e os de
  //    controle. Trocados por '_' em vez de removidos: assim dois arquivos
  //    diferentes nao viram o mesmo nome por acidente.
  String limpo;
  limpo.reserve(n.length());
  for (size_t i = 0; i < n.length(); ++i) {
    const char c = n[i];
    const bool proibido = (c == '\\' || c == '/' || c == ':' || c == '*' || c == '?'
                        || c == '"'  || c == '<' || c == '>' || c == '|');
    limpo += (proibido || (uint8_t)c < 0x20) ? '_' : c;
  }
  n = limpo;

  // 5. Nomes que nao sao nome.
  if (n.length() == 0 || n == "." || n == "..") {
    char alternativo[40];
    snprintf(alternativo, sizeof(alternativo), "RECEBIDO_%lu.nc", (unsigned long)millis());
    return String(alternativo);
  }

  // 6. Teto de tamanho, preservando a extensao -- e a extensao que diz ao
  //    operador o que e o arquivo.
  const size_t kMax = 64;
  if (n.length() > kMax) {
    const int ponto = n.lastIndexOf('.');
    String ext = (ponto > 0 && (n.length() - ponto) <= 8) ? n.substring(ponto) : String("");
    n = n.substring(0, kMax - ext.length()) + ext;
  }

  return n;
}

size_t StorageManager::escreverTudo(File& destino, const uint8_t* dados, size_t n,
                                    SemaphoreHandle_t mutex) {
  size_t total = 0;
  uint8_t semProgresso = 0;

  while (total < n) {
    if (mutex) xSemaphoreTake(mutex, portMAX_DELAY);
    const size_t gravados = destino.write(dados + total, n - total);
    if (mutex) xSemaphoreGive(mutex);

    if (gravados == 0) {
      // ~250 ms sem NENHUM progresso: ai e cartao com defeito, nao ocupacao
      // passageira. Desiste e deixa quem chamou decidir o que fazer.
      if (++semProgresso > 50) {
        logEventf("ERRO", "Cartao parou de aceitar escrita: %u de %u bytes",
                  (unsigned)total, (unsigned)n);
        return total;
      }
      // Quem chama esta sob o cao de guarda; esperar sem alimenta-lo trocaria
      // um arquivo perdido por um reinicio do equipamento.
      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    if (gravados < n - total) {
      // Nao e erro, mas precisa aparecer: virando rotina, o cartao esta no fim
      // da vida e o proximo passo e troca-lo, nao mexer no codigo.
      logEventf("INFO", "Escrita parcial no cartao (%u de %u) - continuando",
                (unsigned)gravados, (unsigned)(n - total));
    }
    semProgresso = 0;
    total += gravados;
  }
  return total;
}

bool StorageManager::ensureBaseDirs() {
  if (!_ok) return false;
  makeDir(DIR_PROGRAMS);
  makeDir(DIR_RECEIVED);
  makeDir(DIR_BACKUPS);
  makeDir(DIR_LOGS);
  makeDir(DIR_TMP);
  return true;
}

bool StorageManager::makeDir(const String& path) {
  if (!_ok) return false;
  String p = sanitizePath(path);
  bool ok;
  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  ok = SD.exists(p) || SD.mkdir(p);
  xSemaphoreGive(_sdMutex);
  return ok;
}

bool StorageManager::exists(const String& path) {
  if (!_ok) return false;
  String p = sanitizePath(path);
  bool ok;
  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  ok = SD.exists(p);
  xSemaphoreGive(_sdMutex);
  return ok;
}

uint64_t StorageManager::fileSize(const String& path) {
  if (!_ok) return 0;
  String p = sanitizePath(path);
  uint64_t size = 0;
  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  File f = SD.open(p, FILE_READ);
  if (f && !f.isDirectory()) size = f.size();
  if (f) f.close();
  xSemaphoreGive(_sdMutex);
  return size;
}

// ESTES DOIS ENTRAVAM NO DRIVER SEM O MUTEX.
//
// Todo o resto da classe serializa o acesso ao cartao; so `totalBytes` e
// `usedBytes` nao serializavam -- e sao justamente os mais chamados de fora:
// a descoberta na rede responde com o espaco livre a cada consulta, o portal
// mostra na tela de estado e o IDENTIFY do protocolo manda para o aplicativo.
//
// Ou seja: tres tarefas diferentes cutucando o driver do cartao ao mesmo tempo
// em que uma transferencia legitima o usa com o mutex na mao. Acesso
// concorrente a um driver que nao e reentrante nao da erro: da travamento.
// ESPERA CURTA, NUNCA portMAX_DELAY. QUEM PERGUNTA ISTO E TAREFA DE REDE.
//
// Espaco livre e informacao de TELA: a descoberta responde com ele, o portal
// mostra, o IDENTIFY manda para o aplicativo. Nenhum dos tres pode ficar preso
// esperando o cartao -- e foi exatamente o que aconteceu quando estes metodos
// passaram a pegar o mutex com espera infinita: uma tentativa de montar um
// cartao ausente segura o mutex por dezenas de segundos, e nesse tempo a caixa
// sumia da rede inteira. De fora parecia queda de Wi-Fi; o ping continuava
// respondendo, porque quem responde ping e a pilha, nao as nossas tarefas.
//
// Devolver 0 quando o cartao esta ocupado e a resposta CERTA aqui: a tela
// mostra um numero desatualizado por um instante, em vez de o equipamento
// inteiro parar de atender. Quem precisa do dado de verdade -- ler e gravar
// arquivo -- continua esperando o quanto for, com portMAX_DELAY.
static const TickType_t kEsperaCurtaDoCartao = pdMS_TO_TICKS(50);

uint64_t StorageManager::totalBytes() {
  if (!_ok || !_sdMutex) return 0;
  if (xSemaphoreTake(_sdMutex, kEsperaCurtaDoCartao) != pdTRUE) return 0;
  const uint64_t v = SD.totalBytes();
  xSemaphoreGive(_sdMutex);
  return v;
}

uint64_t StorageManager::usedBytes() {
  if (!_ok || !_sdMutex) return 0;
  if (xSemaphoreTake(_sdMutex, kEsperaCurtaDoCartao) != pdTRUE) return 0;
  const uint64_t v = SD.usedBytes();
  xSemaphoreGive(_sdMutex);
  return v;
}

uint64_t StorageManager::freeBytes() {
  uint64_t t = totalBytes();
  uint64_t u = usedBytes();
  return t > u ? t - u : 0;
}

String StorageManager::listJson(const String& path) {
  String p = sanitizePath(path);
  String json = "{\"path\":\"" + escapeJson(p) + "\",\"items\":[";
  if (!_ok) return json + "]}";

  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  File root = SD.open(p);
  if (root && root.isDirectory()) {
    bool first = true;
    File file = root.openNextFile();
    while (file) {
      String name = String(file.name());
      int slash = name.lastIndexOf('/');
      if (slash >= 0) name = name.substring(slash + 1);
      if (!first) json += ",";
      first = false;
      json += "{\"name\":\"" + escapeJson(name) + "\",";
      json += "\"path\":\"" + escapeJson(String(file.path())) + "\",";
      json += "\"dir\":" + String(file.isDirectory() ? "true" : "false") + ",";
      json += "\"size\":" + String((uint64_t)file.size()) + "}";
      file.close();
      file = root.openNextFile();
    }
  }
  if (root) root.close();
  xSemaphoreGive(_sdMutex);
  json += "]}";
  return json;
}

bool StorageManager::removeRecursive(const String& path) {
  File f = SD.open(path);
  if (!f) return false;
  if (!f.isDirectory()) {
    f.close();
    return SD.remove(path);
  }
  File child = f.openNextFile();
  while (child) {
    String cp = String(child.path());
    child.close();
    removeRecursive(cp);
    child = f.openNextFile();
  }
  f.close();
  return SD.rmdir(path);
}

bool StorageManager::removePath(const String& path) {
  if (!_ok) return false;
  String p = sanitizePath(path);
  if (p == "/") return false;
  bool ok;
  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  ok = removeRecursive(p);
  xSemaphoreGive(_sdMutex);
  return ok;
}

bool StorageManager::renamePath(const String& from, const String& to) {
  if (!_ok) return false;
  String a = sanitizePath(from);
  String b = sanitizePath(to);
  bool ok;
  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  ok = SD.rename(a, b);
  xSemaphoreGive(_sdMutex);
  return ok;
}

bool StorageManager::copyFile(const String& from, const String& to) {
  if (!_ok) return false;
  String a = sanitizePath(from);
  String b = sanitizePath(to);
  bool ok = false;
  uint8_t buf[512];
  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  File src = SD.open(a, FILE_READ);
  File dst = SD.open(b, FILE_WRITE);
  if (src && dst && !src.isDirectory()) {
    ok = true;
    while (src.available()) {
      size_t n = src.read(buf, sizeof(buf));
      if (escreverTudo(dst, buf, n) != n) { ok = false; break; }
      delay(0);
    }
  }
  if (src) src.close();
  if (dst) dst.close();
  xSemaphoreGive(_sdMutex);
  return ok;
}

bool StorageManager::movePath(const String& from, const String& to) {
  return renamePath(from, to);
}
