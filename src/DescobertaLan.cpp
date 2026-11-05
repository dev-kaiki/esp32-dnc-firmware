#include "DescobertaLan.h"

#include <WiFi.h>

namespace smi {
namespace dnc {
namespace esp {

void DescobertaLan::begin(const String& nome, const String& versao, uint16_t portaDoProtocolo)
{
    m_nome = nome;
    m_versao = versao;
    m_portaProtocolo = portaDoProtocolo;

    // Escuta na porta de descoberta em todas as interfaces. Vale tanto na rede
    // da empresa quanto no ponto de acesso proprio da caixa -- o que importa,
    // porque a primeira configuracao acontece justamente no AP proprio, antes
    // de a caixa ter entrado em rede nenhuma.
    m_iniciado = m_udp.begin(kPortaDescoberta) == 1;
}

void DescobertaLan::laco()
{
    if (!m_iniciado) return;

    const int tamanho = m_udp.parsePacket();
    if (tamanho <= 0) return;

    const int lidos = m_udp.read(m_buffer, sizeof(m_buffer) - 1);
    if (lidos <= 0) return;
    m_buffer[lidos] = '\0';

    // SO RESPONDE A PERGUNTA CERTA.
    //
    // Rede de fabrica e cheia de trafego de difusao -- impressora, DHCP,
    // descoberta de servico de meia duzia de fabricantes. Responder a tudo seria
    // ruido nosso somado ao deles, e ainda entregaria a existencia do
    // equipamento a qualquer varredura.
    if (strncmp(m_buffer, kPerguntaDescoberta, strlen(kPerguntaDescoberta)) != 0) return;

    const bool ocupada = (m_ocupado && m_ocupado());
    const uint64_t livre = m_espacoLivre ? m_espacoLivre() : 0;

    const bool emAp = (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA)
                   && WiFi.status() != WL_CONNECTED;
    const IPAddress meuIp = emAp ? WiFi.softAPIP() : WiFi.localIP();

    // Resposta em texto, campos separados por '|'.
    //
    // Texto e nao binario de proposito: esta mensagem e a primeira coisa que
    // alguem inspeciona quando "o aplicativo nao acha a caixa", e poder ler o
    // pacote num analisador de rede sem decodificador nenhum vale mais do que
    // os poucos bytes que se economizaria.
    String resposta;
    resposta.reserve(160);
    resposta += "SMI-DNC|";
    resposta += m_nome;            resposta += '|';
    resposta += meuIp.toString();  resposta += '|';
    resposta += String(m_portaProtocolo); resposta += '|';
    resposta += m_versao;          resposta += '|';
    resposta += (ocupada ? "1" : "0"); resposta += '|';
    resposta += String((unsigned long)(livre / 1024));   // KB

    // RESPOSTA EM UNICAST, direto para quem perguntou.
    //
    // E o detalhe que faz isto funcionar no Android sem pedir trava de
    // multicast ao sistema: do lado do aplicativo chega um pacote UDP comum.
    m_udp.beginPacket(m_udp.remoteIP(), m_udp.remotePort());
    m_udp.write(reinterpret_cast<const uint8_t*>(resposta.c_str()), resposta.length());
    m_udp.endPacket();
}

} // namespace esp
} // namespace dnc
} // namespace smi
