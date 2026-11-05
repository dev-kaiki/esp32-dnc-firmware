#pragma once

// ---------------------------------------------------------------------------
// Protocolo SMI DNC — Android <-> ESP32
// ---------------------------------------------------------------------------
//
// ESTE ARQUIVO É COMPILADO PELOS DOIS LADOS. Não inclua Qt aqui, nem Arduino.
//
// É o mesmo truque usado em core/licensing/LicenseCrypto.cpp, que o aplicativo
// e o gerador de licenças compartilham: enquanto for UM arquivo, os dois lados
// não têm como divergir. Codec duplicado é onde nasce o bug que só aparece
// quando o quadro atravessa a rede — e aí o log de um lado diz uma coisa e o do
// outro diz outra.
//
// Dependências permitidas: <cstdint>, <cstddef>, <vector>, <string>.
//
// ---------------------------------------------------------------------------
// FORMATO DO QUADRO
// ---------------------------------------------------------------------------
//
//   offset  tam  campo
//   ------  ---  ---------------------------------------------------------
//        0    1  'S'
//        1    1  'M'
//        2    1  versão do protocolo
//        3    1  tipo da mensagem
//        4    1  flags
//        5    4  opId    (little-endian)
//        9    4  seq     (little-endian)
//       13    4  tamanho do payload (little-endian)
//       17    N  payload
//     17+N    4  CRC32 do cabeçalho + payload (little-endian)
//
// Total = 21 + N bytes.
//
// Little-endian fixo no fio: tanto o ESP32 quanto os ARM do Android são
// little-endian, mas gravar isso explicitamente evita que a primeira máquina
// big-endian que aparecer quebre em silêncio.
//
// ---------------------------------------------------------------------------
// POR QUE opId
// ---------------------------------------------------------------------------
//
// Numa rede sem fio o app pode reenviar um comando que na verdade CHEGOU — a
// resposta é que se perdeu. Sem identificador, "iniciar envio" executaria duas
// vezes e a máquina receberia o programa em dobro.
//
// Todo comando crítico carrega um opId. O ESP32 guarda os últimos vistos e
// responde ao repetido com o MESMO resultado, sem executar de novo.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace smi {
namespace dnc {
namespace proto {

// --- Constantes do formato ---------------------------------------------------

constexpr uint8_t  kMagic0          = 'S';
constexpr uint8_t  kMagic1          = 'M';
constexpr uint8_t  kVersaoAtual     = 1;
constexpr size_t   kTamanhoCabecalho = 17;
constexpr size_t   kTamanhoCrc      = 4;
constexpr size_t   kTamanhoMinimo   = kTamanhoCabecalho + kTamanhoCrc;

// Teto do payload.
//
// Não é arbitrário: é o tamanho do bloco de upload, e ele precisa caber com
// folga na RAM do ESP32 junto com os buffers de TX/RX da serial. 4 KB dá
// vazão boa no Wi-Fi sem espremer o que a UART precisa.
constexpr uint32_t kMaxPayload      = 4096;

// Teto defensivo do buffer do decodificador. Se o fluxo virar lixo, o
// decodificador não pode crescer sem limite esperando um quadro que nunca vem.
constexpr size_t   kMaxBufferDecod  = (kTamanhoMinimo + kMaxPayload) * 4;

// --- Tipos de mensagem -------------------------------------------------------

enum Tipo : uint8_t {
    // Sessão
    TIPO_HELLO          = 0x01,  // app -> esp   abre a conversa
    TIPO_IDENTIFY       = 0x02,  // esp -> app   versão, firmware, capacidades
    TIPO_CLAIM          = 0x03,  // app -> esp   assume o controle
    TIPO_RELEASE        = 0x04,  // app -> esp   devolve o controle
    TIPO_PING           = 0x05,  // ambos        prova de vida
    TIPO_PONG           = 0x06,

    // Configuração
    TIPO_CONFIG_SET     = 0x10,  // app -> esp   parâmetros da serial e do envio
    TIPO_CONFIG_GET     = 0x11,

    // Rede sem fio — para instalar uma caixa NOVA sem técnico e sem navegador.
    //
    // A caixa que não entra em rede nenhuma sobe o próprio ponto de acesso
    // (SMI_DNC_ESP32, 192.168.4.1). O tablete entra nessa rede, acha a caixa
    // pela descoberta normal, e o IDENTIFY já diz que ela está sem rede
    // cadastrada — é por aí que o aplicativo sabe que é a primeira conexão.
    // Daí ele pede a lista do que a caixa enxerga e manda a escolha do cliente.
    //
    // Pelo protocolo, e não pelo portal web, para o operador não precisar de
    // navegador, endereço de IP nem senha de administrador do equipamento.
    TIPO_WIFI_SCAN      = 0x12,  // app -> esp   liste as redes que você enxerga
    TIPO_WIFI_LIST      = 0x13,  // esp -> app   u8 quantas · por rede: texto ssid, i8 sinal, u8 protegida
    TIPO_WIFI_SET       = 0x14,  // app -> esp   texto ssid · texto senha

    // Transferência de arquivo (app -> esp)
    TIPO_FILE_BEGIN     = 0x20,  // nome, tamanho total, sha256 esperado
    TIPO_FILE_BLOCK     = 0x21,  // seq = número do bloco
    TIPO_FILE_END       = 0x22,  // fecha e valida
    TIPO_FILE_ABORT     = 0x23,  // descarta o temporário

    // --- Sentido inverso: a caixa MANDA um arquivo para o aplicativo ---------
    //
    // POR QUE PRECISOU EXISTIR: o protocolo só tinha subida. Receber programa
    // da CNC funcionava e o arquivo ficava no cartão da caixa, sem nenhum
    // caminho até o tablete — o operador via "recebido" e não tinha o programa.
    //
    // O desenho espelha a subida, inclusive na conferência: o conteúdo só vale
    // depois que tamanho e hash batem. Programa de CNC corrompido em silêncio é
    // pior do que transferência que falha.
    TIPO_FILE_GET       = 0x24,  // app -> esp   pede um arquivo pelo nome
    TIPO_FILE_INFO      = 0x25,  // esp -> app   nome e tamanho, antes dos dados
    TIPO_FILE_DATA      = 0x26,  // esp -> app   um bloco (seq = numero do bloco)
    TIPO_FILE_DATA_END  = 0x27,  // esp -> app   fim, com o sha256 do que foi lido

    // Gerenciar o cartão PELO APLICATIVO, sem portal e sem navegador.
    //
    // Os programas enviados ficam gravados no cartão e nada os apaga sozinho.
    // Sem isto, o operador não tem como ver o que ocupa espaço nem liberar
    // espaço quando encher -- e descobriria o cartão cheio só na hora em que um
    // envio fosse recusado, no meio do turno.
    TIPO_FILE_LIST      = 0x28,  // app -> esp   texto pasta ("" = /programs)
    TIPO_FILE_LIST_RESP = 0x29,  // esp -> app   u64 livre · u64 total · u16 quantos · por arquivo: texto nome, u32 bytes
    TIPO_FILE_DELETE    = 0x2A,  // app -> esp   texto pasta · texto nome

    // Operações com a CNC
    TIPO_OP_START       = 0x30,  // payload diz: envio único, drip ou recepção
    TIPO_OP_PAUSE       = 0x31,
    TIPO_OP_RESUME      = 0x32,
    TIPO_OP_CANCEL      = 0x33,

    // Respostas do operador aos avisos que mantêm a porta ABERTA.
    //
    // POR QUE NÃO DÁ PARA REUSAR OS DE CIMA: cancelar descarta o que resta;
    // retomar só tira a pausa. Nenhum dos dois é "a máquina terminou de usinar,
    // pode fechar" nem "continue esperando, ela ainda está processando". Usar
    // OP_CANCEL para confirmar o fim -- que era o que o backend fazia --
    // registra um encerramento normal como cancelamento e joga fora o que
    // estivesse na fila.
    TIPO_OP_FINISH      = 0x34,  // app -> esp   fim confirmado pelo operador
    TIPO_OP_CONTINUE    = 0x35,  // app -> esp   continuar aguardando a máquina

    // Consulta
    TIPO_STATE_QUERY    = 0x40,

    // Respostas
    TIPO_ACK            = 0x50,
    TIPO_NACK           = 0x51,

    // Eventos (esp -> app), sem resposta
    TIPO_EVENT_STATE    = 0x60,
    TIPO_EVENT_PROGRESS = 0x61,
    TIPO_EVENT_LOG      = 0x62,
    TIPO_EVENT_ERROR    = 0x63,
    TIPO_EVENT_SIGNALS  = 0x64,
};

// --- Flags -------------------------------------------------------------------

enum Flag : uint8_t {
    FLAG_NENHUMA      = 0x00,
    FLAG_EXIGE_ACK    = 0x01,   // remetente espera ACK/NACK
    FLAG_REPETICAO    = 0x02,   // reenvio de um comando já mandado
};

// --- Motivos de NACK ---------------------------------------------------------
//
// Número, não texto: o app decide o que dizer ao operador, no idioma dele.
enum MotivoNack : uint8_t {
    NACK_OK               = 0,
    NACK_CRC              = 1,   // quadro corrompido
    NACK_VERSAO           = 2,   // versão de protocolo incompatível
    NACK_ESTADO_INVALIDO  = 3,   // comando não cabe no estado atual
    NACK_SEM_CONTROLE     = 4,   // outro dispositivo tem o controle
    NACK_BLOCO_FORA_ORDEM = 5,
    NACK_ARQUIVO_INVALIDO = 6,   // tamanho ou hash não bateram
    NACK_ARMAZENAMENTO    = 7,   // falha no microSD
    NACK_PARAMETRO        = 8,   // payload malformado
    NACK_OCUPADO          = 9,
    NACK_NAO_SUPORTADO    = 10,
};

// --- Quadro ------------------------------------------------------------------

struct Quadro {
    uint8_t  versao = kVersaoAtual;
    uint8_t  tipo   = 0;
    uint8_t  flags  = FLAG_NENHUMA;
    uint32_t opId   = 0;
    uint32_t seq    = 0;
    std::vector<uint8_t> payload;
};

// --- CRC32 (IEEE 802.3, refletido) ------------------------------------------
uint32_t crc32(const uint8_t* dados, size_t tamanho, uint32_t inicial = 0xFFFFFFFFu);

// --- Codificação -------------------------------------------------------------
//
// Devolve o quadro pronto para ir ao socket. Payload acima de kMaxPayload é
// recusado devolvendo vetor vazio — melhor falhar aqui que estourar do outro
// lado.
std::vector<uint8_t> codificar(const Quadro& q);

// Atalhos usados o tempo todo.
std::vector<uint8_t> codificarAck(uint32_t opId);
std::vector<uint8_t> codificarNack(uint32_t opId, MotivoNack motivo);

// --- Decodificação -----------------------------------------------------------

enum ResultadoDecod : uint8_t {
    DECOD_VAZIO = 0,      // ainda não há quadro completo no buffer
    DECOD_QUADRO,         // saiu um quadro válido
    DECOD_CRC_INVALIDO,   // quadro completo, CRC errado — descartado
    DECOD_LIXO,           // bytes descartados procurando o início
    DECOD_ESTOURO,        // buffer excedeu o teto; foi limpo
};

// Decodificador de fluxo.
//
// TCP entrega pedaço, não mensagem: um quadro pode chegar em cinco leituras, e
// cinco quadros podem chegar numa só. Esta classe acumula e devolve um quadro
// por vez.
class Decodificador {
public:
    void alimentar(const uint8_t* dados, size_t tamanho);

    // Extrai o próximo quadro. Chame em laço até receber DECOD_VAZIO.
    ResultadoDecod proximo(Quadro& saida);

    void limpar();
    size_t pendente() const { return m_buffer.size(); }

    // Contadores para diagnóstico — o app mostra na tela de Diagnóstico.
    uint32_t quadrosOk() const      { return m_quadrosOk; }
    uint32_t errosCrc() const       { return m_errosCrc; }
    uint32_t bytesDescartados() const { return m_bytesDescartados; }

private:
    // Move o início do buffer até o próximo par de bytes mágicos.
    // Devolve quantos bytes foram jogados fora.
    size_t sincronizar();

    std::vector<uint8_t> m_buffer;
    uint32_t m_quadrosOk = 0;
    uint32_t m_errosCrc = 0;
    uint32_t m_bytesDescartados = 0;
};

// --- Controle de duplicidade -------------------------------------------------
//
// Guarda os últimos opId executados. Comando repetido é reconhecido sem
// executar de novo — ver a nota no topo sobre por que isso existe.
class ControleDeDuplicidade {
public:
    explicit ControleDeDuplicidade(size_t capacidade = 32);

    // true  = opId inédito, pode executar (fica registrado)
    // false = já foi executado; devolva a resposta anterior
    bool registrarSeNovo(uint32_t opId);

    bool jaVisto(uint32_t opId) const;
    void limpar();

private:
    std::vector<uint32_t> m_vistos;
    size_t m_capacidade;
    size_t m_proximo = 0;
};

// --- Ajudantes de payload ----------------------------------------------------
//
// Payload é binário; estes ajudantes evitam erro de deslocamento espalhado
// pelo código dos dois lados.
void escreverU8 (std::vector<uint8_t>& p, uint8_t v);
void escreverU16(std::vector<uint8_t>& p, uint16_t v);
void escreverU32(std::vector<uint8_t>& p, uint32_t v);
void escreverU64(std::vector<uint8_t>& p, uint64_t v);
void escreverTexto(std::vector<uint8_t>& p, const std::string& s);   // u16 tam + bytes
void escreverBytes(std::vector<uint8_t>& p, const uint8_t* d, size_t n);

// Leitores. `pos` avança; devolvem false se faltar byte — nunca leem além do
// fim, que é como um payload truncado vira travamento.
bool lerU8 (const std::vector<uint8_t>& p, size_t& pos, uint8_t& v);
bool lerU16(const std::vector<uint8_t>& p, size_t& pos, uint16_t& v);
bool lerU32(const std::vector<uint8_t>& p, size_t& pos, uint32_t& v);
bool lerU64(const std::vector<uint8_t>& p, size_t& pos, uint64_t& v);
bool lerTexto(const std::vector<uint8_t>& p, size_t& pos, std::string& s);

const char* nomeDoTipo(uint8_t tipo);
const char* nomeDoMotivo(uint8_t motivo);

} // namespace proto
} // namespace dnc
} // namespace smi
