#include "DncProtocolServer.h"
#include <algorithm>

#include "../Config.h"
#include "StorageManager.h"
#include "Logger.h"

// Para alimentar o cao enquanto espera o cartao liberar.
#include <esp_task_wdt.h>

#include <mbedtls/sha256.h>

namespace smi {
namespace dnc {
namespace esp {

namespace {
// Nome fixo do temporário. Um só, porque só há um upload por vez — o núcleo
// recusa FILE_BEGIN fora de IDLE/READY.
const char* kTemporario = "/upload.tmp";

// Progresso no máximo a cada 200 ms.
//
// A CNC é alimentada por uma tarefa de tempo real; encher o Wi-Fi de eventos
// roubaria CPU dela. O olho do operador também não distingue mais que cinco
// atualizações por segundo.
constexpr uint32_t kIntervaloProgressoMs = 200;

// Silencio maximo tolerado de um cliente antes de considera-lo morto.
//
// O aplicativo manda PING a cada 3 segundos enquanto esta vivo. Vinte segundos
// sao mais de seis PINGs perdidos -- nao acontece por lentidao de rede, so por
// morte do outro lado.
//
// Nao da para confiar em `connected()` para isso: um socket meio-aberto continua
// se declarando conectado por horas. Sem este prazo, a caixa ficava presa
// segurando um cliente morto e recusando o aplicativo de verdade.
constexpr uint32_t kSilencioMaxDoClienteMs = 20000;
} // namespace

// ---------------------------------------------------------------------------
// ArmazenamentoSD
// ---------------------------------------------------------------------------

void ArmazenamentoSD::travar()
{
    if (m_mutex) xSemaphoreTake(m_mutex, portMAX_DELAY);
}

void ArmazenamentoSD::destravar()
{
    if (m_mutex) xSemaphoreGive(m_mutex);
}

bool ArmazenamentoSD::abrirTemporario()
{
    descartarTemporario();

    travar();
    m_arquivo = SD.open(kTemporario, FILE_WRITE);
    const bool ok = (bool)m_arquivo;
    destravar();
    if (!ok) return false;

    auto* ctx = new mbedtls_sha256_context;
    mbedtls_sha256_init(ctx);
    // 0 = SHA-256 (1 seria SHA-224).
    if (mbedtls_sha256_starts(ctx, 0) != 0) {
        mbedtls_sha256_free(ctx);
        delete ctx;
        return false;
    }
    m_ctxSha = ctx;
    m_escritos = 0;
    m_aberto = true;
    return true;
}

bool ArmazenamentoSD::escreverTemporario(const uint8_t* dados, size_t n)
{
    if (!m_aberto || !m_ctxSha) return false;

    // ESCRITA PARCIAL NO CARTAO NAO E FALHA. INSISTIR NO QUE FALTOU.
    //
    // MEDIDO EM CAMPO: "programa nao aceito pelo SD", de forma ALEATORIA -- o
    // mesmo arquivo ora passava ora nao. A versao anterior fazia uma unica
    // chamada a write() e tratava QUALQUER retorno menor que o pedido como erro
    // fatal, abortando o upload inteiro:
    //
    //     const size_t gravados = m_arquivo.write(dados, n);
    //     if (gravados != n) return false;
    //
    // Mas escrita curta e comportamento normal de cartao: num instante ele
    // aceita menos (apagamento de bloco, atualizacao de FAT, disputa pelo
    // barramento com a tarefa que le o programa para a maquina) e no instante
    // seguinte aceita o resto. Era isso -- e nao o arquivo -- que decidia se o
    // envio passava.
    //
    // O caminho da SERIAL ja tratava este mesmo caso corretamente ("se a UART
    // aceitou menos do que espiamos, o resto continua na fila"); o do cartao
    // nao. Duas regras diferentes para o mesmo problema.
    //
    // So desiste depois de ~250 ms SEM NENHUM progresso, que a essa altura e
    // cartao com defeito de verdade, nao ocupacao passageira.
    size_t total = 0;
    uint8_t semProgresso = 0;
    while (total < n) {
        travar();
        const size_t gravados = m_arquivo.write(dados + total, n - total);
        destravar();

        if (gravados == 0) {
            if (++semProgresso > 50) {
                logEventf("ERRO", "Cartao parou de aceitar escrita: %u de %u bytes "
                                  "gravados antes de desistir", (unsigned)total, (unsigned)n);
                return false;
            }
            // A tarefa que chama aqui esta sob o cao de guarda; esperar sem
            // alimenta-lo trocaria um upload perdido por um reinicio.
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        if (gravados < n - total) {
            // Nao e erro, mas precisa aparecer: se isto virar rotina, o cartao
            // esta no fim da vida e o proximo passo e troca-lo, nao mexer no
            // codigo.
            logEventf("INFO", "Escrita parcial no cartao (%u de %u) - continuando",
                      (unsigned)gravados, (unsigned)(n - total));
        }
        semProgresso = 0;
        total += gravados;
    }

    // Hash na passagem: no fim não precisa reler o cartão.
    if (mbedtls_sha256_update(static_cast<mbedtls_sha256_context*>(m_ctxSha), dados, n) != 0) {
        return false;
    }
    m_escritos += n;
    return true;
}

bool ArmazenamentoSD::fecharTemporario()
{
    if (!m_aberto) return false;
    travar();
    m_arquivo.flush();
    m_arquivo.close();
    destravar();
    m_aberto = false;
    return true;
}

bool ArmazenamentoSD::validarTemporario(const uint8_t hashEsperado[32], uint64_t tamanhoEsperado)
{
    if (!m_ctxSha) return false;
    if (m_escritos != tamanhoEsperado) return false;

    uint8_t calculado[32] = {0};
    const int r = mbedtls_sha256_finish(static_cast<mbedtls_sha256_context*>(m_ctxSha), calculado);

    mbedtls_sha256_free(static_cast<mbedtls_sha256_context*>(m_ctxSha));
    delete static_cast<mbedtls_sha256_context*>(m_ctxSha);
    m_ctxSha = nullptr;

    if (r != 0) return false;
    return memcmp(calculado, hashEsperado, 32) == 0;
}

uint64_t ArmazenamentoSD::espacoTotal() const
{
    // Direto do cartao, como espacoLivre() ao lado -- e sob a mesma trava, que
    // e o que impede duas tarefas de falarem com o cartao ao mesmo tempo.
    auto* self = const_cast<ArmazenamentoSD*>(this);
    self->travar();
    const uint64_t total = SD.totalBytes();
    self->destravar();
    return total;
}

std::vector<IArmazenamento::ArquivoNoCartao>
ArmazenamentoSD::listar(const std::string& pasta)
{
    std::vector<ArquivoNoCartao> fora;

    // Pasta vazia = a de programas. E o caso normal e evita o aplicativo ter de
    // conhecer os caminhos internos do cartao.
    String dir = pasta.empty() ? String(m_pasta) : String(pasta.c_str());
    if (!dir.startsWith("/")) dir = "/" + dir;

    travar();
    File d = SD.open(dir);
    if (d && d.isDirectory()) {
        File f = d.openNextFile();
        while (f) {
            if (!f.isDirectory()) {
                ArquivoNoCartao a;
                a.nome  = std::string(f.name());
                // Nome vem com caminho em algumas versoes da biblioteca; a tela
                // quer so o nome.
                const size_t barra = a.nome.find_last_of('/');
                if (barra != std::string::npos) a.nome = a.nome.substr(barra + 1);
                a.bytes = (uint32_t)f.size();
                fora.push_back(a);
            }
            f.close();
            f = d.openNextFile();
        }
    }
    if (d) d.close();
    destravar();

    // MAIOR PRIMEIRO: quem abre esta tela esta procurando o que liberar espaco,
    // nao o que foi gravado por ultimo.
    std::sort(fora.begin(), fora.end(),
              [](const ArquivoNoCartao& a, const ArquivoNoCartao& b) {
                  return a.bytes > b.bytes;
              });
    return fora;
}

bool ArmazenamentoSD::apagar(const std::string& pasta, const std::string& nome)
{
    // O MESMO SANEADOR DE SEMPRE: nome vindo da rede nunca vira caminho.
    const String limpo = StorageManager::sanitizeFileName(String(nome.c_str()));
    if (limpo.length() == 0) return false;

    String dir = pasta.empty() ? String(m_pasta) : String(pasta.c_str());
    if (!dir.startsWith("/")) dir = "/" + dir;

    // So as pastas de programa e de recebidos. Sem isto, um pedido podeira
    // apagar registro, configuracao ou o proprio firmware guardado no cartao.
    if (dir != String(DIR_PROGRAMS) && dir != String(DIR_RECEIVED)) return false;

    const String caminho = dir + "/" + limpo;
    travar();
    const bool ok = SD.exists(caminho) && SD.remove(caminho);
    destravar();
    return ok;
}

bool ArmazenamentoSD::promoverTemporario(const std::string& nomeFinal)
{
    // O MESMO SANEADOR DE QUEM VAI PROCURAR O ARQUIVO DEPOIS.
    //
    // Aqui havia uma copia reduzida da regra: tirava caminho, mas nao truncava
    // em 64 caracteres como o saneador de verdade. Um programa de nome longo
    // era gravado com o nome inteiro e procurado com o nome cortado -- e o
    // envio morria em "Arquivo nao encontrado no microSD", que na tela vira um
    // "Falha" mudo. Duas regras para o mesmo nome e um defeito esperando a vez.
    const String nome = StorageManager::sanitizeFileName(String(nomeFinal.c_str()));
    if (nome.length() == 0) return false;

    const String destino = m_pasta + "/" + nome;

    travar();
    if (SD.exists(destino)) SD.remove(destino);
    const bool ok = SD.rename(kTemporario, destino);
    destravar();
    return ok;
}

void ArmazenamentoSD::descartarTemporario()
{
    if (m_aberto) {
        travar();
        m_arquivo.close();
        destravar();
        m_aberto = false;
    }
    if (m_ctxSha) {
        mbedtls_sha256_free(static_cast<mbedtls_sha256_context*>(m_ctxSha));
        delete static_cast<mbedtls_sha256_context*>(m_ctxSha);
        m_ctxSha = nullptr;
    }
    m_escritos = 0;

    travar();
    if (SD.exists(kTemporario)) SD.remove(kTemporario);
    destravar();
}

bool ArmazenamentoSD::abrirParaLeitura(const std::string& nome, uint64_t& tamanho)
{
    fecharLeitura();

    // Mesma regra da gravacao, pela mesma razao: o nome tem que casar com o que
    // foi gravado, e um pedido com ".." nao pode ler fora da pasta.
    const String limpo = StorageManager::sanitizeFileName(String(nome.c_str()));
    if (limpo.length() == 0) return false;

    // PROCURA NOS DOIS LUGARES, E NOS RECEBIDOS PRIMEIRO.
    //
    // O caso que existe na pratica e o operador acabar de receber um programa
    // da maquina e querer ele no aparelho -- esse esta em /received. A pasta de
    // programas vem depois, para tambem dar para buscar de volta algo que ja
    // foi enviado. Nome repetido nas duas: ganha o recebido, que e o mais novo
    // e o que foi pedido.
    travar();
    bool ok = false;
    const String candidatos[2] = { m_pastaRecebidos + "/" + limpo,
                                   m_pasta + "/" + limpo };
    for (const String& caminho : candidatos) {
        m_leitura = SD.open(caminho, FILE_READ);
        if (m_leitura && !m_leitura.isDirectory()) { ok = true; break; }
        if (m_leitura) m_leitura.close();
    }
    if (ok) tamanho = (uint64_t)m_leitura.size();
    destravar();

    if (!ok) return false;

    auto* ctx = new mbedtls_sha256_context;
    mbedtls_sha256_init(ctx);
    if (mbedtls_sha256_starts(ctx, 0) != 0) {
        mbedtls_sha256_free(ctx);
        delete ctx;
        travar(); m_leitura.close(); destravar();
        return false;
    }
    m_ctxShaLeitura = ctx;
    m_lendo = true;
    return true;
}

size_t ArmazenamentoSD::lerPedaco(uint8_t* destino, size_t maximo)
{
    if (!m_lendo) return 0;

    travar();
    const int lidos = m_leitura.read(destino, maximo);
    destravar();

    if (lidos <= 0) return 0;

    // Hash na passagem: no fim nao precisa reler o cartao.
    if (m_ctxShaLeitura) {
        mbedtls_sha256_update(static_cast<mbedtls_sha256_context*>(m_ctxShaLeitura),
                              destino, (size_t)lidos);
    }
    return (size_t)lidos;
}

bool ArmazenamentoSD::hashDaLeitura(uint8_t saida[32])
{
    if (!m_ctxShaLeitura) return false;
    return mbedtls_sha256_finish(
        static_cast<mbedtls_sha256_context*>(m_ctxShaLeitura), saida) == 0;
}

void ArmazenamentoSD::fecharLeitura()
{
    if (m_lendo) {
        travar();
        m_leitura.close();
        destravar();
        m_lendo = false;
    }
    if (m_ctxShaLeitura) {
        mbedtls_sha256_free(static_cast<mbedtls_sha256_context*>(m_ctxShaLeitura));
        delete static_cast<mbedtls_sha256_context*>(m_ctxShaLeitura);
        m_ctxShaLeitura = nullptr;
    }
}


uint64_t ArmazenamentoSD::espacoLivre() const
{
    auto* self = const_cast<ArmazenamentoSD*>(this);
    self->travar();
    const uint64_t total = SD.totalBytes();
    const uint64_t usado = SD.usedBytes();
    self->destravar();
    return total > usado ? (total - usado) : 0;
}

// ---------------------------------------------------------------------------
// TransporteWiFi
// ---------------------------------------------------------------------------

void TransporteWiFi::enviar(const uint8_t* dados, size_t n)
{
    if (!m_cliente || !m_cliente->connected()) return;
    m_cliente->write(dados, n);
}

bool TransporteWiFi::conectado() const
{
    return m_cliente && m_cliente->connected();
}

// ---------------------------------------------------------------------------
// AcoesFirmware
// ---------------------------------------------------------------------------

std::vector<IAcoesDnc::RedeVista> AcoesFirmware::redesVisiveis()
{
    if (!aoVarrerRedes) return {};
    return aoVarrerRedes();
}

bool AcoesFirmware::gravarRede(const std::string& ssid, const std::string& senha)
{
    if (!aoGravarRede) return false;
    return aoGravarRede(ssid, senha);
}

bool AcoesFirmware::semRedeCadastrada()
{
    // Sem ninguem para perguntar, a resposta honesta e "nao sei" -- e dizer
    // que FALTA rede faria o aplicativo abrir a tela de instalacao numa caixa
    // que ja esta funcionando.
    if (!aoPerguntarSeTemRede) return false;
    return aoPerguntarSeTemRede();
}

bool AcoesFirmware::aplicarConfigSerial(const ConfigSerial& c)
{
    // Sem ninguem ligado do outro lado, nao da para afirmar que foi aplicada.
    if (!aoAplicarConfig) return false;
    return aoAplicarConfig(c);
}

bool AcoesFirmware::iniciarOperacao(uint8_t modo, const std::string& nomeArquivo)
{
    if (!aoIniciar) return false;
    return aoIniciar(modo, String(nomeArquivo.c_str()));
}

void AcoesFirmware::pausarOperacao()  { if (aoPausar) aoPausar(); }
void AcoesFirmware::retomarOperacao() { if (aoRetomar) aoRetomar(); }
void AcoesFirmware::cancelarOperacao(){ if (aoCancelar) aoCancelar(); }
void AcoesFirmware::finalizarOperacao()   { if (aoFinalizar) aoFinalizar(); }
void AcoesFirmware::continuarAguardando() { if (aoContinuarAguardando) aoContinuarAguardando(); }

// ---------------------------------------------------------------------------
// DncProtocolServer
// ---------------------------------------------------------------------------

DncProtocolServer::DncProtocolServer()
    : m_servidor(0), m_core(m_arm, m_trans, m_acoes) {}

void DncProtocolServer::begin(uint16_t porta,
                              SemaphoreHandle_t mutexSD,
                              const String& pastaProgramas,
                              const String& firmware,
                              const String& idAparelho,
                              uint8_t linhasDisponiveis)
{
    m_arm.definirMutex(mutexSD);
    m_arm.definirPastaDeProgramas(pastaProgramas);
    m_arm.definirPastaDeRecebidos(DIR_RECEIVED);
    m_trans.definirCliente(&m_cliente);

    m_core.definirIdentidade(std::string(firmware.c_str()),
                             std::string(idAparelho.c_str()),
                             linhasDisponiveis);

    m_servidor = WiFiServer(porta);
    m_servidor.begin();
    m_servidor.setNoDelay(true);   // sem Nagle: comando pequeno não pode esperar
    m_iniciado = true;
}

void DncProtocolServer::laco()
{
    if (!m_iniciado) return;

    // UM CLIENTE POR VEZ.
    //
    // O núcleo já recusa um segundo controlador pelo CLAIM, mas aceitar o
    // socket e depois negar deixaria a conexão pendurada consumindo memória.
    // Recusar na porta é mais barato e mais honesto.
    const uint32_t agora = millis();

    // DADO ESPERANDO E PROVA DE VIDA, mesmo sem termos lido ainda.
    //
    // ISTO CORRIGE UM ERRO GRAVE DE MEDIDA. O prazo abaixo media "ha quanto
    // tempo EU LI bytes", e nao "ha quanto tempo o CLIENTE FALOU". Sao coisas
    // diferentes quando esta tarefa deixa de rodar por um tempo -- e isso
    // acontecia: a recepcao segurava o mutex do cartao, a TaskWebServer travava
    // na primeira consulta de espaco livre, e este laco parava junto, porque e a
    // mesma tarefa. Os PINGs chegavam e ficavam na fila do soquete, sem serem
    // lidos. Ao voltar a rodar, o prazo estourado derrubava uma conexao VIVA.
    //
    // Quanto mais longo o programa, mais vezes isso acontecia -- exatamente
    // quando derrubar a conexao dói mais.
    if (m_cliente && m_cliente.connected() && m_cliente.available() > 0) {
        m_ultimaFalaDoClienteMs = agora;
    }

    // CLIENTE CALADO DEMAIS E CLIENTE MORTO.
    //
    // Precisa vir ANTES de aceitar o proximo: e o que libera a vaga para o
    // aplicativo que esta tentando entrar. Ver a nota em kSilencioMaxDoClienteMs.
    if (m_cliente && m_cliente.connected()
        && (agora - m_ultimaFalaDoClienteMs) > kSilencioMaxDoClienteMs) {
        m_cliente.stop();
        m_core.aoPerderConexao();
    }

    if (m_servidor.hasClient()) {
        WiFiClient novo = m_servidor.accept();
        if (m_cliente && m_cliente.connected()) {
            novo.stop();
        } else {
            m_cliente = novo;
            m_cliente.setNoDelay(true);
            // Comeca o relogio na entrada: sem isto, um cliente que conecta e
            // nunca fala seria julgado pelo tempo do cliente ANTERIOR.
            m_ultimaFalaDoClienteMs = agora;
        }
    }

    if (m_cliente && !m_cliente.connected()) {
        m_cliente.stop();
        m_core.aoPerderConexao();
    }

    // UM BLOCO DE DOWNLOAD POR VOLTA.
    //
    // Fica aqui, junto do resto do laco, para o arquivo subir sem travar o OTA,
    // a descoberta nem os comandos do operador. Ver a nota no nucleo.
    m_core.continuarEnvioDeArquivo();

    while (m_cliente && m_cliente.available()) {
        const int lidos = m_cliente.read(m_buffer, sizeof(m_buffer));
        if (lidos <= 0) break;
        m_ultimaFalaDoClienteMs = agora;
        m_core.aoReceberBytes(m_buffer, static_cast<size_t>(lidos));
    }
}

void DncProtocolServer::publicarEstado(uint8_t estado)
{
    m_core.definirEstado(estado);
}

void DncProtocolServer::publicarProgresso(uint64_t enviados, uint64_t total, uint32_t bps)
{
    const uint32_t agora = millis();
    if (agora - m_ultimoProgressoMs < kIntervaloProgressoMs) return;
    m_ultimoProgressoMs = agora;
    m_core.notificarProgresso(enviados, total, bps);
}

void DncProtocolServer::publicarSinais(uint8_t bitmask)
{
    m_core.notificarSinais(bitmask);
}

void DncProtocolServer::publicarLog(const String& texto)
{
    m_core.notificarLog(std::string(texto.c_str()));
}

} // namespace esp
} // namespace dnc
} // namespace smi
