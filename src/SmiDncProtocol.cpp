#include "SmiDncProtocol.h"

#include <cstring>

namespace smi {
namespace dnc {
namespace proto {

// ---------------------------------------------------------------------------
// CRC32
// ---------------------------------------------------------------------------
//
// Implementação por bits, sem tabela. É ~8 operações por byte, contra 1 da
// versão com tabela de 1 KB.
//
// A escolha é pelo ESP32: 1 KB de RAM estática vale mais aqui do que o ganho
// de CPU. Com quadros de 4 KB a 240 MHz, o custo do CRC é ordens de grandeza
// menor que o próprio Wi-Fi. Se um dia o perfil apontar isto como gargalo — e
// eu duvido — a tabela entra sem mudar a assinatura.

uint32_t crc32(const uint8_t* dados, size_t tamanho, uint32_t inicial)
{
    uint32_t crc = inicial;
    for (size_t i = 0; i < tamanho; ++i) {
        crc ^= dados[i];
        for (int b = 0; b < 8; ++b) {
            const uint32_t mascara = (crc & 1u) ? 0xEDB88320u : 0u;
            crc = (crc >> 1) ^ mascara;
        }
    }
    return crc;
}

namespace {

inline void porU32(std::vector<uint8_t>& v, uint32_t x)
{
    v.push_back(static_cast<uint8_t>( x        & 0xFF));
    v.push_back(static_cast<uint8_t>((x >>  8) & 0xFF));
    v.push_back(static_cast<uint8_t>((x >> 16) & 0xFF));
    v.push_back(static_cast<uint8_t>((x >> 24) & 0xFF));
}

inline uint32_t pegaU32(const uint8_t* p)
{
    return  static_cast<uint32_t>(p[0])
         | (static_cast<uint32_t>(p[1]) <<  8)
         | (static_cast<uint32_t>(p[2]) << 16)
         | (static_cast<uint32_t>(p[3]) << 24);
}

} // namespace

// ---------------------------------------------------------------------------
// Codificação
// ---------------------------------------------------------------------------

std::vector<uint8_t> codificar(const Quadro& q)
{
    // Recusa aqui, não no outro lado. Um payload grande demais que sai daqui
    // vira um NACK remoto e uma investigação; recusado na origem, vira um bug
    // visível na hora em quem chamou.
    if (q.payload.size() > kMaxPayload) return {};

    const uint32_t tam = static_cast<uint32_t>(q.payload.size());

    std::vector<uint8_t> saida;
    saida.reserve(kTamanhoMinimo + q.payload.size());

    saida.push_back(kMagic0);
    saida.push_back(kMagic1);
    saida.push_back(q.versao);
    saida.push_back(q.tipo);
    saida.push_back(q.flags);
    porU32(saida, q.opId);
    porU32(saida, q.seq);
    porU32(saida, tam);

    if (tam > 0) saida.insert(saida.end(), q.payload.begin(), q.payload.end());

    const uint32_t crc = crc32(saida.data(), saida.size());
    porU32(saida, crc);

    return saida;
}

std::vector<uint8_t> codificarAck(uint32_t opId)
{
    Quadro q;
    q.tipo = TIPO_ACK;
    q.opId = opId;
    return codificar(q);
}

std::vector<uint8_t> codificarNack(uint32_t opId, MotivoNack motivo)
{
    Quadro q;
    q.tipo = TIPO_NACK;
    q.opId = opId;
    escreverU8(q.payload, static_cast<uint8_t>(motivo));
    return codificar(q);
}

// ---------------------------------------------------------------------------
// Decodificação
// ---------------------------------------------------------------------------

void Decodificador::alimentar(const uint8_t* dados, size_t tamanho)
{
    if (!dados || tamanho == 0) return;
    m_buffer.insert(m_buffer.end(), dados, dados + tamanho);
}

void Decodificador::limpar()
{
    m_buffer.clear();
}

size_t Decodificador::sincronizar()
{
    // Procura o par mágico a partir do byte 1. O byte 0 já foi rejeitado por
    // quem chamou, então recomeçar dele daria laço infinito.
    for (size_t i = 1; i + 1 < m_buffer.size(); ++i) {
        if (m_buffer[i] == kMagic0 && m_buffer[i + 1] == kMagic1) {
            m_buffer.erase(m_buffer.begin(), m_buffer.begin() + static_cast<long>(i));
            return i;
        }
    }

    // Nenhum início à vista. Guarda só o último byte: ele pode ser o 'S' de um
    // quadro cujo 'M' ainda não chegou.
    const size_t descartados = m_buffer.empty() ? 0 : m_buffer.size() - 1;
    if (m_buffer.size() > 1) {
        m_buffer.erase(m_buffer.begin(), m_buffer.end() - 1);
    }
    return descartados;
}

ResultadoDecod Decodificador::proximo(Quadro& saida)
{
    // Teto do buffer. Fluxo virou lixo: limpa e avisa, em vez de crescer até a
    // memória acabar. No ESP32 isso seria reinício por falta de heap.
    if (m_buffer.size() > kMaxBufferDecod) {
        m_bytesDescartados += static_cast<uint32_t>(m_buffer.size());
        m_buffer.clear();
        return DECOD_ESTOURO;
    }

    if (m_buffer.size() < 2) return DECOD_VAZIO;

    if (m_buffer[0] != kMagic0 || m_buffer[1] != kMagic1) {
        const size_t n = sincronizar();
        m_bytesDescartados += static_cast<uint32_t>(n);
        return DECOD_LIXO;
    }

    if (m_buffer.size() < kTamanhoCabecalho) return DECOD_VAZIO;

    const uint32_t tam = pegaU32(m_buffer.data() + 13);

    // Tamanho absurdo significa cabeçalho corrompido, não quadro gigante.
    // Sem esta guarda, ficaríamos esperando para sempre por bytes que não vêm.
    if (tam > kMaxPayload) {
        m_buffer[0] = 0;                       // invalida o início
        const size_t n = sincronizar();
        m_bytesDescartados += static_cast<uint32_t>(n);
        ++m_errosCrc;
        return DECOD_CRC_INVALIDO;
    }

    const size_t total = kTamanhoCabecalho + tam + kTamanhoCrc;
    if (m_buffer.size() < total) return DECOD_VAZIO;   // ainda chegando

    const uint32_t crcEsperado = pegaU32(m_buffer.data() + kTamanhoCabecalho + tam);
    const uint32_t crcCalc = crc32(m_buffer.data(), kTamanhoCabecalho + tam);

    if (crcCalc != crcEsperado) {
        ++m_errosCrc;
        m_buffer[0] = 0;
        const size_t n = sincronizar();
        m_bytesDescartados += static_cast<uint32_t>(n);
        return DECOD_CRC_INVALIDO;
    }

    saida.versao = m_buffer[2];
    saida.tipo   = m_buffer[3];
    saida.flags  = m_buffer[4];
    saida.opId   = pegaU32(m_buffer.data() + 5);
    saida.seq    = pegaU32(m_buffer.data() + 9);
    saida.payload.assign(m_buffer.begin() + static_cast<long>(kTamanhoCabecalho),
                         m_buffer.begin() + static_cast<long>(kTamanhoCabecalho + tam));

    m_buffer.erase(m_buffer.begin(), m_buffer.begin() + static_cast<long>(total));
    ++m_quadrosOk;
    return DECOD_QUADRO;
}

// ---------------------------------------------------------------------------
// Controle de duplicidade
// ---------------------------------------------------------------------------

ControleDeDuplicidade::ControleDeDuplicidade(size_t capacidade)
    : m_capacidade(capacidade == 0 ? 1 : capacidade)
{
    m_vistos.assign(m_capacidade, 0);
}

bool ControleDeDuplicidade::jaVisto(uint32_t opId) const
{
    if (opId == 0) return false;   // 0 = "sem identificador", nunca deduplica
    for (size_t i = 0; i < m_vistos.size(); ++i) {
        if (m_vistos[i] == opId) return true;
    }
    return false;
}

bool ControleDeDuplicidade::registrarSeNovo(uint32_t opId)
{
    if (opId == 0) return true;    // eventos e consultas não são deduplicados
    if (jaVisto(opId)) return false;

    m_vistos[m_proximo] = opId;
    m_proximo = (m_proximo + 1) % m_vistos.size();
    return true;
}

void ControleDeDuplicidade::limpar()
{
    m_vistos.assign(m_capacidade, 0);
    m_proximo = 0;
}

// ---------------------------------------------------------------------------
// Ajudantes de payload
// ---------------------------------------------------------------------------

void escreverU8(std::vector<uint8_t>& p, uint8_t v) { p.push_back(v); }

void escreverU16(std::vector<uint8_t>& p, uint16_t v)
{
    p.push_back(static_cast<uint8_t>( v       & 0xFF));
    p.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

void escreverU32(std::vector<uint8_t>& p, uint32_t v) { porU32(p, v); }

void escreverU64(std::vector<uint8_t>& p, uint64_t v)
{
    porU32(p, static_cast<uint32_t>(v & 0xFFFFFFFFu));
    porU32(p, static_cast<uint32_t>((v >> 32) & 0xFFFFFFFFu));
}

void escreverBytes(std::vector<uint8_t>& p, const uint8_t* d, size_t n)
{
    if (d && n) p.insert(p.end(), d, d + n);
}

void escreverTexto(std::vector<uint8_t>& p, const std::string& s)
{
    // Teto de 65535 pelo próprio campo de tamanho. Nome de arquivo e mensagem
    // de log estão muito abaixo disso; truncar é melhor que gravar um tamanho
    // que não corresponde ao conteúdo.
    const size_t n = s.size() > 0xFFFF ? 0xFFFF : s.size();
    escreverU16(p, static_cast<uint16_t>(n));
    if (n) p.insert(p.end(), s.begin(), s.begin() + static_cast<long>(n));
}

bool lerU8(const std::vector<uint8_t>& p, size_t& pos, uint8_t& v)
{
    if (pos + 1 > p.size()) return false;
    v = p[pos];
    pos += 1;
    return true;
}

bool lerU16(const std::vector<uint8_t>& p, size_t& pos, uint16_t& v)
{
    if (pos + 2 > p.size()) return false;
    v = static_cast<uint16_t>(p[pos] | (static_cast<uint16_t>(p[pos + 1]) << 8));
    pos += 2;
    return true;
}

bool lerU32(const std::vector<uint8_t>& p, size_t& pos, uint32_t& v)
{
    if (pos + 4 > p.size()) return false;
    v = pegaU32(p.data() + pos);
    pos += 4;
    return true;
}

bool lerU64(const std::vector<uint8_t>& p, size_t& pos, uint64_t& v)
{
    if (pos + 8 > p.size()) return false;
    const uint32_t baixo = pegaU32(p.data() + pos);
    const uint32_t alto  = pegaU32(p.data() + pos + 4);
    v = (static_cast<uint64_t>(alto) << 32) | baixo;
    pos += 8;
    return true;
}

bool lerTexto(const std::vector<uint8_t>& p, size_t& pos, std::string& s)
{
    uint16_t n = 0;
    if (!lerU16(p, pos, n)) return false;
    if (pos + n > p.size()) return false;
    s.assign(reinterpret_cast<const char*>(p.data() + pos), n);
    pos += n;
    return true;
}

// ---------------------------------------------------------------------------
// Nomes para log
// ---------------------------------------------------------------------------

const char* nomeDoTipo(uint8_t tipo)
{
    switch (tipo) {
    case TIPO_HELLO:          return "HELLO";
    case TIPO_IDENTIFY:       return "IDENTIFY";
    case TIPO_CLAIM:          return "CLAIM";
    case TIPO_RELEASE:        return "RELEASE";
    case TIPO_PING:           return "PING";
    case TIPO_PONG:           return "PONG";
    case TIPO_CONFIG_SET:     return "CONFIG_SET";
    case TIPO_CONFIG_GET:     return "CONFIG_GET";
    case TIPO_FILE_BEGIN:     return "FILE_BEGIN";
    case TIPO_FILE_BLOCK:     return "FILE_BLOCK";
    case TIPO_FILE_END:       return "FILE_END";
    case TIPO_FILE_ABORT:     return "FILE_ABORT";
    case TIPO_FILE_GET:       return "FILE_GET";
    case TIPO_FILE_INFO:      return "FILE_INFO";
    case TIPO_FILE_DATA:      return "FILE_DATA";
    case TIPO_FILE_DATA_END:  return "FILE_DATA_END";
    case TIPO_OP_START:       return "OP_START";
    case TIPO_OP_PAUSE:       return "OP_PAUSE";
    case TIPO_OP_RESUME:      return "OP_RESUME";
    case TIPO_OP_CANCEL:      return "OP_CANCEL";
    case TIPO_STATE_QUERY:    return "STATE_QUERY";
    case TIPO_ACK:            return "ACK";
    case TIPO_NACK:           return "NACK";
    case TIPO_EVENT_STATE:    return "EVENT_STATE";
    case TIPO_EVENT_PROGRESS: return "EVENT_PROGRESS";
    case TIPO_EVENT_LOG:      return "EVENT_LOG";
    case TIPO_EVENT_ERROR:    return "EVENT_ERROR";
    case TIPO_EVENT_SIGNALS:  return "EVENT_SIGNALS";
    default:                  return "DESCONHECIDO";
    }
}

const char* nomeDoMotivo(uint8_t motivo)
{
    switch (motivo) {
    case NACK_OK:               return "ok";
    case NACK_CRC:              return "quadro corrompido";
    case NACK_VERSAO:           return "versao de protocolo incompativel";
    case NACK_ESTADO_INVALIDO:  return "comando invalido neste estado";
    case NACK_SEM_CONTROLE:     return "outro dispositivo tem o controle";
    case NACK_BLOCO_FORA_ORDEM: return "bloco fora de ordem";
    case NACK_ARQUIVO_INVALIDO: return "tamanho ou hash nao conferem";
    case NACK_ARMAZENAMENTO:    return "falha no cartao microSD";
    case NACK_PARAMETRO:        return "parametro invalido";
    case NACK_OCUPADO:          return "ocupado";
    case NACK_NAO_SUPORTADO:    return "nao suportado por este hardware";
    default:                    return "motivo desconhecido";
    }
}

} // namespace proto
} // namespace dnc
} // namespace smi
