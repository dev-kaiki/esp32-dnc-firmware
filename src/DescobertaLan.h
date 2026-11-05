#pragma once

// ---------------------------------------------------------------------------
// Descoberta na rede local
// ---------------------------------------------------------------------------
//
// O PROBLEMA QUE ISTO RESOLVE
//
// Sem isto, o operador precisa saber o IP da caixa. E o IP muda: num único dia
// de bancada o endereço do PC mudou três vezes, a caixa sumiu ao ser levada
// para outra sala, e reinstalar o aplicativo apagou o endereço guardado duas
// vezes. Numa fábrica com várias caixas, ninguém decora nem gerencia isso.
//
// COMO FUNCIONA
//
// O aplicativo manda UMA pergunta em difusão para a porta abaixo. Cada caixa
// que ouvir responde DIRETO para quem perguntou, dizendo quem é. O aplicativo
// junta as respostas por um segundo e mostra a lista; o operador toca na que
// quer. Endereço nenhum é digitado.
//
// POR QUE NÃO mDNS
//
// Seria a escolha "de manual", e foi descartada por três motivos práticos:
//
//   1. Rede de empresa costuma bloquear multicast entre segmentos — e o
//      sintoma seria "não acha a caixa", sem explicação.
//   2. No Android, RECEBER multicast exige uma trava específica do sistema.
//      Esquecê-la é a origem clássica de "funciona no meu aparelho e não no do
//      cliente". Aqui a resposta volta em UNICAST: o Android a recebe como um
//      pacote normal, sem trava nenhuma.
//   3. Depender de resolvedor mDNS no tablete acrescenta uma peça que pode
//      faltar. Difusão UDP funciona em qualquer pilha de rede.
//
// O QUE ESTA CAMADA NÃO FAZ
//
// Não comanda nada. Responde quem sou, onde estou e se estou ocupada — e só.
// Todo comando continua passando pelo protocolo autenticado da porta 3333, com
// controle exclusivo. Descobrir uma caixa não dá poder algum sobre ela.

#include <Arduino.h>
#include <WiFiUdp.h>
#include <functional>

namespace smi {
namespace dnc {
namespace esp {

// Porta da descoberta. Separada da porta de comando de propósito: quem procura
// ainda não tem controle de nada.
static const uint16_t kPortaDescoberta = 3334;

// Assinatura da pergunta. Curta e específica para não responder a tráfego de
// difusão alheio, que numa rede de fábrica é constante.
static const char kPerguntaDescoberta[] = "SMI-DNC-QUEM-E?";

class DescobertaLan {
public:
    // `nome`, `versao` e `idPlaca` compõem a resposta. `porta` é a do protocolo
    // de comando, para o aplicativo não precisar adivinhar.
    void begin(const String& nome, const String& versao, uint16_t portaDoProtocolo);

    // Chamar em laço. Não bloqueia.
    void laco();

    // Fonte da verdade sobre estar ocupada. Uma caixa alimentando uma CNC
    // aparece na lista marcada como ocupada — assim o operador não interrompe
    // o trabalho de outro por engano.
    void definirVerificadorDeOcupado(std::function<bool()> f) { m_ocupado = std::move(f); }

    // Espaço livre no cartão, em bytes, para a lista já mostrar se cabe o
    // programa.
    void definirEspacoLivre(std::function<uint64_t()> f) { m_espacoLivre = std::move(f); }

    bool iniciado() const { return m_iniciado; }

private:
    WiFiUDP m_udp;
    String  m_nome;
    String  m_versao;
    uint16_t m_portaProtocolo = 0;
    bool m_iniciado = false;

    std::function<bool()>     m_ocupado;
    std::function<uint64_t()> m_espacoLivre;

    char m_buffer[64];
};

} // namespace esp
} // namespace dnc
} // namespace smi
