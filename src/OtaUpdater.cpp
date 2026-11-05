#include "OtaUpdater.h"

#include <ArduinoOTA.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

namespace smi {
namespace dnc {
namespace esp {

bool OtaUpdater::begin(const String& nomeNaRede, const String& senha)
{
    // SEM SENHA NÃO LIGA.
    //
    // Recusar é a resposta certa. Uma caixa acessível na rede da fábrica sem
    // proteção significa que qualquer um regrava o firmware do equipamento que
    // alimenta a CNC — e o estrago disso não é perder configuração, é uma
    // máquina recebendo programa de origem desconhecida.
    if (senha.length() == 0) {
        if (m_log) m_log("OTA NAO habilitado: senha vazia.");
        return false;
    }

    // Sem Wi-Fi não há como oferecer OTA. Não é erro — é ordem de partida: a
    // tarefa de rede chama isto de novo quando a conexão subir.
    if (WiFi.status() != WL_CONNECTED && WiFi.softAPgetStationNum() == 0
        && WiFi.getMode() == WIFI_OFF) {
        return false;
    }

    ArduinoOTA.setHostname(nomeNaRede.c_str());
    ArduinoOTA.setPassword(senha.c_str());

    ArduinoOTA.onStart([this]() {
        m_atualizando = true;
        // Ver a nota sobre o watchdog em onProgress.
        esp_task_wdt_reset();
        if (m_log) m_log("OTA iniciado. Nao desligue o equipamento.");
    });

    ArduinoOTA.onEnd([this]() {
        m_atualizando = false;
        if (m_log) m_log("OTA concluido. Reiniciando.");
    });

    ArduinoOTA.onProgress([this](unsigned int atual, unsigned int total) {
        // Alimentar o cão aqui virou INOFENSIVO, não mais obrigatório.
        //
        // Isto existia porque a gravação prende a tarefa por mais de um minuto
        // e o watchdog reiniciava o ESP32 no meio de toda atualização. Hoje a
        // TaskWebServer -- que é quem roda isto -- NÃO está mais registrada no
        // cão (ver a nota nela: ela travava servindo o log e derrubava a caixa
        // no meio de uma usinagem). Então a chamada abaixo não encontra a
        // tarefa e não faz nada.
        //
        // Fica porque é barata e porque, se alguém um dia devolver esta tarefa
        // ao cão, a gravação volta a precisar dela — e a falta seria descoberta
        // do pior jeito, com OTA reiniciando no meio.
        esp_task_wdt_reset();

        // Um registro a cada 10%: log a cada bloco encheria a fila e roubaria
        // tempo da tarefa que alimenta a máquina.
        static int ultimoPct = -1;
        const int pct = total ? int((atual * 100ULL) / total) : 0;
        if (pct / 10 != ultimoPct / 10) {
            ultimoPct = pct;
            if (m_log) m_log("OTA " + String(pct) + "%");
        }
    });

    ArduinoOTA.onError([this](ota_error_t erro) {
        m_atualizando = false;
        String t = "OTA falhou: ";
        switch (erro) {
        case OTA_AUTH_ERROR:    t += "senha incorreta"; break;
        case OTA_BEGIN_ERROR:   t += "sem espaco na particao de OTA"; break;
        case OTA_CONNECT_ERROR: t += "conexao recusada"; break;
        case OTA_RECEIVE_ERROR: t += "falha ao receber"; break;
        case OTA_END_ERROR:     t += "falha ao finalizar"; break;
        default:                t += "erro desconhecido"; break;
        }
        if (m_log) m_log(t);
    });

    ArduinoOTA.begin();
    m_iniciado = true;
    if (m_log) m_log("OTA disponivel como '" + nomeNaRede + "'.");
    return true;
}

void OtaUpdater::laco()
{
    if (!m_iniciado) return;

    // ATUALIZAR COM A MÁQUINA RECEBENDO É INACEITÁVEL.
    //
    // O OTA reinicia o ESP32 ao terminar. Fazer isso no meio de um envio
    // deixaria a CNC com o programa pela metade — o mesmo estrago que a
    // correção da drenagem evitou do lado do aplicativo.
    //
    // Simplesmente não atendemos o OTA enquanto há operação em curso: quem
    // tentar gravar recebe recusa de conexão e tenta de novo depois.
    if (m_ocupado && m_ocupado() && !m_atualizando) return;

    ArduinoOTA.handle();
}

} // namespace esp
} // namespace dnc
} // namespace smi
