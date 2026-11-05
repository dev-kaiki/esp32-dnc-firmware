#pragma once

// ---------------------------------------------------------------------------
// Shell Arduino do servidor de protocolo
// ---------------------------------------------------------------------------
//
// Aqui mora tudo o que depende do Arduino: WiFiServer, cartão SD e a fila de
// comandos do firmware. A LÓGICA não está aqui — está em DncProtocolCore, que
// é C++ puro e tem 38 testes rodando fora do aparelho.
//
// Essa divisão é o que permite provar o comportamento sem gravar no ESP32 a
// cada mudança. Este arquivo é fino de propósito: se ele crescer com decisões,
// é sinal de que alguma coisa deveria ter ido para o núcleo.
//
// O QUE ESTE ARQUIVO NÃO FAZ: não toca na UART da CNC. A comunicação com a
// máquina continua inteira no TaskSerial/TaskDNC existentes. O servidor só
// enfileira comandos — do mesmo jeito que o portal web já fazia.

#include <Arduino.h>
#include <SD.h>
#include <WiFi.h>
#include <functional>

#include "DncProtocolCore.h"

namespace smi {
namespace dnc {
namespace esp {

// --- Armazenamento em microSD ------------------------------------------------
//
// Monta o arquivo num TEMPORÁRIO e só renomeia para o nome definitivo depois
// de conferidos tamanho e hash. Ver a nota em IArmazenamento.
class ArmazenamentoSD : public IArmazenamento {
public:
    // `mutexSD` é o mesmo do StorageManager: o cartão é compartilhado com as
    // outras tarefas, e duas escritas simultâneas corrompem o sistema de
    // arquivos.
    void definirMutex(SemaphoreHandle_t mutexSD) { m_mutex = mutexSD; }
    void definirPastaDeProgramas(const String& pasta) { m_pasta = pasta; }

    // ONDE O PROGRAMA RECEBIDO DA MAQUINA FICA.
    //
    // A gravacao do upload vai para a pasta de programas, mas o que vem da CNC
    // e gravado em outra -- para o que veio da maquina nao sobrescrever o que
    // o operador preparou. Sem contar esta pasta ao leitor, o download recusava
    // exatamente o arquivo que o operador acabou de receber.
    void definirPastaDeRecebidos(const String& pasta) { m_pastaRecebidos = pasta; }

    bool abrirTemporario() override;
    bool escreverTemporario(const uint8_t* dados, size_t n) override;
    bool fecharTemporario() override;
    bool validarTemporario(const uint8_t hashEsperado[32], uint64_t tamanhoEsperado) override;
    bool promoverTemporario(const std::string& nomeFinal) override;
    void descartarTemporario() override;
    uint64_t espacoLivre() const override;
    uint64_t espacoTotal() const override;
    std::vector<ArquivoNoCartao> listar(const std::string& pasta) override;
    bool apagar(const std::string& pasta, const std::string& nome) override;

    // Leitura, para mandar um arquivo de volta ao aplicativo.
    bool abrirParaLeitura(const std::string& nome, uint64_t& tamanho) override;
    size_t lerPedaco(uint8_t* destino, size_t maximo) override;
    bool hashDaLeitura(uint8_t saida[32]) override;
    void fecharLeitura() override;

private:
    void travar();
    void destravar();

    SemaphoreHandle_t m_mutex = nullptr;
    String m_pasta = "/programs";
    String m_pastaRecebidos = "/received";
    File m_arquivo;
    bool m_aberto = false;

    // Hash calculado ENQUANTO grava, não relendo o arquivo depois.
    //
    // Reler 1 MB do cartão para conferir custaria segundos e um segundo buffer
    // de RAM. Calculando na passagem, a validação no FILE_END é instantânea.
    void* m_ctxSha = nullptr;
    uint64_t m_escritos = 0;

    // Leitura. Arquivo e hash separados dos de escrita de proposito: um upload
    // e um download podem coexistir, e misturar os contextos corromperia os dois.
    File  m_leitura;
    bool  m_lendo = false;
    void* m_ctxShaLeitura = nullptr;
};

// --- Transporte por Wi-Fi ----------------------------------------------------

class TransporteWiFi : public ITransporte {
public:
    void definirCliente(WiFiClient* c) { m_cliente = c; }
    void enviar(const uint8_t* dados, size_t n) override;
    bool conectado() const override;

private:
    WiFiClient* m_cliente = nullptr;
};

// --- Ações: encaminha para a fila do firmware --------------------------------
//
// Não executa nada aqui. Enfileira, exatamente como o portal web já fazia, e
// quem executa continua sendo o TaskDNC. É isso que mantém a UART da máquina
// intocada por esta camada.
class AcoesFirmware : public IAcoesDnc {
public:
    std::function<bool(const ConfigSerial&)> aoAplicarConfig;
    std::function<std::vector<RedeVista>()> aoVarrerRedes;
    std::function<bool(const std::string&, const std::string&)> aoGravarRede;
    std::function<bool()> aoPerguntarSeTemRede;
    std::function<bool(uint8_t, const String&)> aoIniciar;
    std::function<void()> aoPausar;
    std::function<void()> aoRetomar;
    std::function<void()> aoCancelar;
    std::function<void()> aoFinalizar;
    std::function<void()> aoContinuarAguardando;

    bool aplicarConfigSerial(const ConfigSerial& c) override;
    std::vector<RedeVista> redesVisiveis() override;
    bool gravarRede(const std::string& ssid, const std::string& senha) override;
    bool semRedeCadastrada() override;
    bool iniciarOperacao(uint8_t modo, const std::string& nomeArquivo) override;
    void pausarOperacao() override;
    void retomarOperacao() override;
    void cancelarOperacao() override;
    void finalizarOperacao() override;
    void continuarAguardando() override;
};

// --- Servidor ----------------------------------------------------------------

class DncProtocolServer {
public:
    DncProtocolServer();

    void begin(uint16_t porta,
               SemaphoreHandle_t mutexSD,
               const String& pastaProgramas,
               const String& firmware,
               const String& idAparelho,
               uint8_t linhasDisponiveis);

    // Chamado em laço pela tarefa de rede. Não bloqueia.
    void laco();

    // Eventos empurrados pelo firmware para o app.
    void publicarEstado(uint8_t estado);
    void publicarProgresso(uint64_t enviados, uint64_t total, uint32_t bps);
    void publicarSinais(uint8_t bitmask);
    void publicarLog(const String& texto);

    AcoesFirmware& acoes() { return m_acoes; }
    uint8_t estado() const { return m_core.estado(); }

    // NÃO é `const` de propósito.
    //
    // No core ESP32 3.3.10 tanto `NetworkClient::connected()` quanto o
    // `operator bool()` são métodos NÃO-const. Marcar isto como `const` não
    // compila. A alternativa seria um `const_cast`, mas mentir sobre a
    // constância de um objeto que mexe em socket não paga o que custa.
    bool clienteConectado() { return m_cliente && m_cliente.connected(); }

private:
    WiFiServer m_servidor;
    WiFiClient m_cliente;

    ArmazenamentoSD m_arm;
    TransporteWiFi m_trans;
    AcoesFirmware m_acoes;
    DncProtocolCore m_core;

    bool m_iniciado = false;
    uint32_t m_ultimoProgressoMs = 0;

    // Quando o cliente falou pela ultima vez.
    //
    // POR QUE ISTO PRECISA EXISTIR: `WiFiClient::connected()` continua dizendo
    // "conectado" para um socket meio-aberto -- aquele cujo dono morreu sem
    // avisar (aplicativo fechado a forca, celular dormindo, Wi-Fi caindo). Como
    // so aceitamos UM cliente por vez, a caixa ficava presa segurando um morto e
    // RECUSANDO o aplicativo de verdade para sempre. Na pratica, o equipamento
    // morria ate alguem tirar da tomada. Observado em bancada.
    uint32_t m_ultimaFalaDoClienteMs = 0;

    uint8_t m_buffer[1024];
};

} // namespace esp
} // namespace dnc
} // namespace smi
