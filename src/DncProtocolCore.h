#pragma once

// ---------------------------------------------------------------------------
// Núcleo do servidor de protocolo — SEM Arduino
// ---------------------------------------------------------------------------
//
// POR QUE ISTO É PURO C++ E NÃO USA WiFi, SD OU Serial DIRETO.
//
// Não há arduino-cli nesta máquina, só o compilador xtensa. Sem separar, a
// única forma de saber se a lógica está certa seria gravar no ESP32 e olhar a
// máquina — o que é exatamente o ciclo de depuração que já custou caro neste
// projeto.
//
// Separado, o núcleo é compilado e TESTADO aqui, contra um armazenamento
// falso, e depois cruzado para o alvo. O shell Arduino (DncProtocolServer)
// fica fino o bastante para ser conferido por leitura.
//
// A regra é simples: se precisa de `#include <Arduino.h>`, não entra aqui.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "SmiDncProtocol.h"

namespace smi {
namespace dnc {
namespace esp {

// --- Estados, na ordem definida no protocolo ---------------------------------

enum Estado : uint8_t {
    EST_IDLE = 0,
    EST_RECEIVING_FILE,
    EST_VALIDATING_FILE,
    EST_READY,
    EST_SENDING,
    EST_ONLINE,
    EST_PAUSED,
    EST_RECEIVING_FROM_CNC,
    EST_COMPLETED,
    EST_CANCELLED,
    EST_ERROR,

    // ACRESCENTADOS NO FIM, para nao renumerar o que ja existe.
    //
    // Programa inteiro entregue a porta; a maquina pode ainda estar executando.
    // NAO e conclusao: a porta continua aberta esperando o operador confirmar.
    EST_AWAIT_FINISH,

    // A drenagem final parou de avancar. Tambem NAO encerra nada -- e uma
    // pergunta ao operador, com a porta aberta.
    EST_DRAIN_STALLED,

    // RECEBENDO DA CNC E A MAQUINA PAROU DE MANDAR. A PORTA CONTINUA ABERTA.
    //
    // O espelho do EST_AWAIT_FINISH, no sentido da recepcao. Parametros de
    // Siemens tem pausas longas no meio -- fechar na primeira perde o resto.
    EST_RECEIVE_IDLE,
};

const char* nomeDoEstado(uint8_t e);

// --- Configuração serial recebida do app -------------------------------------

struct ConfigSerial {
    uint32_t baud = 9600;
    uint8_t  dataBits = 8;
    uint8_t  paridade = 0;      // 0 nenhuma · 2 par · 3 impar (valores do Qt)
    uint8_t  stopBits = 1;
    uint8_t  fluxo = 0;         // 0 nenhum · 1 XON/XOFF · 3 RTS/CTS
    bool     dtr = false;
    bool     rts = false;

    uint32_t taxaManualBps = 0;
    uint32_t chunkInicial = 32;
    uint32_t chunkMin = 8;
    uint32_t chunkMax = 96;
    uint32_t holdSemConsumoMs = 250;
    uint32_t debounceMs = 40;
};

// --- Armazenamento -----------------------------------------------------------
//
// Abstrai o microSD. O arquivo é montado num TEMPORÁRIO e só vira definitivo
// depois de conferidos tamanho e hash.
//
// Isso não é preciosismo: uma queda de Wi-Fi no meio do upload não pode deixar
// um arquivo que PARECE válido na pasta de programas. O operador escolheria
// esse arquivo, mandaria para a máquina, e a peça sairia errada.
class IArmazenamento {
public:
    virtual ~IArmazenamento() = default;

    virtual bool abrirTemporario() = 0;
    virtual bool escreverTemporario(const uint8_t* dados, size_t n) = 0;
    virtual bool fecharTemporario() = 0;

    // Confere tamanho E hash do que foi gravado.
    virtual bool validarTemporario(const uint8_t hashEsperado[32],
                                   uint64_t tamanhoEsperado) = 0;

    // Só depois de validar. Substitui o definitivo de forma atômica.
    virtual bool promoverTemporario(const std::string& nomeFinal) = 0;

    virtual void descartarTemporario() = 0;
    virtual uint64_t espacoLivre() const = 0;

    // --- Gerenciar o cartao pelo aplicativo ---------------------------------
    //
    // Os programas gravados nunca sao apagados sozinhos. Sem poder LISTAR e
    // APAGAR pelo aplicativo, o operador so descobre o cartao cheio quando um
    // envio e recusado -- e nao tem como resolver sem navegador.
    struct ArquivoNoCartao {
        std::string nome;
        uint32_t    bytes = 0;
    };
    virtual uint64_t espacoTotal() const = 0;
    virtual std::vector<ArquivoNoCartao> listar(const std::string& pasta) = 0;
    virtual bool apagar(const std::string& pasta, const std::string& nome) = 0;

    // --- Leitura, para mandar um arquivo de volta ao aplicativo --------------
    //
    // Faltava por completo: esta interface so sabia GRAVAR. O programa recebido
    // da CNC ficava no cartao sem nenhum caminho ate o tablete.
    //
    // Devolve o tamanho em `tamanho`, ou false se o arquivo nao existe.
    virtual bool abrirParaLeitura(const std::string& nome, uint64_t& tamanho) = 0;

    // Le o proximo pedaco. Devolve quantos bytes leu (0 = acabou).
    virtual size_t lerPedaco(uint8_t* destino, size_t maximo) = 0;

    // SHA-256 de tudo o que foi lido desde abrirParaLeitura().
    //
    // Calculado NA PASSAGEM, por quem le o cartao: o nucleo e C++ puro e nao tem
    // biblioteca de hash. Reler o arquivo so para conferir custaria o dobro do
    // tempo e um segundo buffer que nao temos.
    virtual bool hashDaLeitura(uint8_t saida[32]) = 0;

    virtual void fecharLeitura() = 0;
};

// --- Saída para o app --------------------------------------------------------

class ITransporte {
public:
    virtual ~ITransporte() = default;
    virtual void enviar(const uint8_t* dados, size_t n) = 0;
    virtual bool conectado() const = 0;
};

// --- Ações que o shell executa ----------------------------------------------
//
// O núcleo decide QUANDO; o shell sabe COMO. Isso mantém a UART fora daqui.
class IAcoesDnc {
public:
    virtual ~IAcoesDnc() = default;

    // Devolve false quando a configuracao e RECUSADA (fora de faixa).
    //
    // Era void, e o nucleo respondia ACK em qualquer caso: a caixa recusava a
    // configuracao em silencio e o aplicativo mostrava "salva e enviada para a
    // caixa". O operador conferia a tela, via o valor novo, e a maquina seguia
    // com o antigo -- divergencia silenciosa, a pior de achar em campo.
    virtual bool aplicarConfigSerial(const ConfigSerial& c) = 0;

    // --- Rede sem fio (instalacao de caixa nova) ---------------------------
    //
    // `redesVisiveis` devolve o que a caixa enxerga agora; `gravarRede` salva e
    // manda a caixa tentar entrar. Separados porque varrer demora segundos e
    // gravar e instantaneo.
    struct RedeVista {
        std::string ssid;
        int8_t      sinal = 0;      // dBm
        bool        protegida = true;
    };
    virtual std::vector<RedeVista> redesVisiveis() = 0;
    virtual bool gravarRede(const std::string& ssid, const std::string& senha) = 0;

    // Verdadeiro quando a caixa esta sem rede cadastrada ou so com o ponto de
    // acesso de socorro no ar -- e o sinal de "primeira instalacao" que o
    // aplicativo le no IDENTIFY.
    virtual bool semRedeCadastrada() = 0;

    // modo: 0 envio único · 1 drip-feed · 2 recepção da CNC
    // Devolve false se não deu para começar (ex.: arquivo sumiu do cartão).
    virtual bool iniciarOperacao(uint8_t modo, const std::string& nomeArquivo) = 0;

    virtual void pausarOperacao() = 0;
    virtual void retomarOperacao() = 0;
    virtual void cancelarOperacao() = 0;

    // O operador confirmou que a maquina terminou de EXECUTAR: pode fechar.
    //
    // NAO E CANCELAR, e a diferenca ja custou caro: o aplicativo mandava
    // TIPO_OP_CANCEL para confirmar o fim, o que registrava um encerramento
    // normal como cancelamento e ainda descartava o que estivesse na fila.
    virtual void finalizarOperacao() = 0;

    // O operador respondeu "continuar aguardando" ao aviso de drenagem travada.
    // Reabre o prazo; sem isso a pergunta voltaria no instante seguinte, num
    // laco que nunca sai do lugar.
    virtual void continuarAguardando() = 0;
};

// --- Núcleo ------------------------------------------------------------------

class DncProtocolCore {
public:
    DncProtocolCore(IArmazenamento& armazenamento,
                    ITransporte& transporte,
                    IAcoesDnc& acoes);

    void definirIdentidade(const std::string& firmware,
                           const std::string& idAparelho,
                           uint8_t linhasDisponiveis);

    // Bytes vindos do socket. Pode ser pedaço de quadro ou vários quadros.
    void aoReceberBytes(const uint8_t* dados, size_t n);

    // O app caiu. Solta o controle, mas NÃO cancela transmissão em andamento —
    // ver a nota na implementação.
    void aoPerderConexao();

    // --- Eventos que o shell empurra para o app ---------------------------
    void notificarEstado(uint8_t estado);
    void notificarProgresso(uint64_t enviados, uint64_t total, uint32_t bps);
    void notificarSinais(uint8_t bitmask);
    void notificarLog(const std::string& texto);
    void notificarErro(uint8_t motivo, const std::string& texto);

    uint8_t estado() const { return m_estado; }
    bool temControlador() const { return m_temControlador; }
    const ConfigSerial& config() const { return m_config; }
    const std::string& arquivoPronto() const { return m_arquivoPronto; }

    // Chamado pelo shell quando a operação termina sozinha.
    void definirEstado(uint8_t novo);

    // Só para teste: quantos comandos foram recusados por estado inválido.
    uint32_t recusasPorEstado() const { return m_recusasPorEstado; }

    // --- Download em andamento ---------------------------------------------
    //
    // Chamar em laço: manda UM bloco por vez, não o arquivo inteiro de uma vez.
    //
    // POR QUE UM POR VEZ: despejar tudo bloquearia esta tarefa por segundos
    // esperando a rede escoar -- e é a mesma tarefa que atende o OTA, a
    // descoberta e os comandos do operador. Um bloco por volta mantém tudo
    // respondendo enquanto o arquivo sobe.
    //
    // Devolve true enquanto houver o que mandar.
    bool continuarEnvioDeArquivo();
    bool enviandoArquivo() const { return m_enviandoArquivo; }

private:
    void tratarQuadro(const proto::Quadro& q);
    void responderAck(uint32_t opId);
    void responderNack(uint32_t opId, proto::MotivoNack motivo);
    void enviarQuadro(const proto::Quadro& q);

    bool comandoCabeNoEstado(uint8_t tipo) const;

    // Nenhuma operacao em curso: da para configurar, subir arquivo e comecar.
    // Ver a nota na implementacao sobre por que o ERRO entra aqui.
    bool foraDeOperacao() const;

    void tratarHello(const proto::Quadro& q);
    void tratarClaim(const proto::Quadro& q);
    void tratarConfigSet(const proto::Quadro& q);
    void tratarWifiScan(const proto::Quadro& q);
    void tratarWifiSet(const proto::Quadro& q);
    void tratarFileList(const proto::Quadro& q);
    void tratarFileDelete(const proto::Quadro& q);
    void tratarFileBegin(const proto::Quadro& q);
    void tratarFileBlock(const proto::Quadro& q);
    void tratarFileEnd(const proto::Quadro& q);
    void tratarOpStart(const proto::Quadro& q);
    void tratarFileGet(const proto::Quadro& q);
    void encerrarEnvioDeArquivo(bool comSucesso);

    IArmazenamento& m_arm;
    ITransporte& m_trans;
    IAcoesDnc& m_acoes;

    proto::Decodificador m_decod;
    proto::ControleDeDuplicidade m_dedup{48};

    std::string m_firmware = "0.0.0";
    std::string m_idAparelho = "SMI-ESP32";
    uint8_t m_linhas = 0;

    uint8_t m_estado = EST_IDLE;
    bool m_temControlador = false;
    std::string m_idControlador;

    ConfigSerial m_config;

    // Upload em andamento.
    std::string m_nomeRecebendo;
    uint64_t m_tamanhoEsperado = 0;
    uint8_t  m_hashEsperado[32] = {0};
    uint64_t m_recebido = 0;
    uint32_t m_proximaSeq = 0;

    std::string m_arquivoPronto;

    // Download em andamento (caixa -> aplicativo).
    bool     m_enviandoArquivo = false;
    uint64_t m_restanteDoEnvio = 0;
    uint32_t m_seqDoEnvio = 0;
    void*    m_shaDoEnvio = nullptr;   // contexto de hash, calculado na passagem

    uint32_t m_recusasPorEstado = 0;
};

} // namespace esp
} // namespace dnc
} // namespace smi
