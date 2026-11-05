#pragma once

// ---------------------------------------------------------------------------
// Atualização de firmware pelo ar (OTA)
// ---------------------------------------------------------------------------
//
// POR QUE EXISTE
//
// Com o MAX3232 ligado na serial, gravar por USB deixa de ser prático — e numa
// caixa instalada dentro do painel da máquina, desmontar para gravar é pior
// ainda. OTA é o que permite atualizar o firmware de um cliente sem ir até lá,
// que é justamente o objetivo do produto.
//
// DUAS LIMITAÇÕES QUE PRECISAM ESTAR CLARAS
//
// 1. A PRIMEIRA GRAVAÇÃO É SEMPRE POR USB. OTA só funciona quando já existe
//    no chip um firmware com OTA rodando e com Wi-Fi de pé. Não há como
//    "entrar em OTA" num chip virgem.
//
// 2. OTA EXIGE PARTIÇÃO COM OTA. O esquema `min_spiffs` (1,9 MB de app)
//    mantém; `huge_app` e `no_ota` NÃO. Se alguém trocar a partição para
//    ganhar espaço, perde a atualização remota junto — e só descobre quando
//    precisar atualizar uma caixa que já está no cliente.
//
// SEGURANÇA
//
// A senha não é opcional. Uma caixa em rede de fábrica sem proteção significa
// que qualquer um na rede regrava o firmware do equipamento que comanda a CNC.
//
// TRANSMISSÃO EM ANDAMENTO
//
// Atualizar durante um envio interromperia a alimentação da máquina no meio do
// programa. O firmware RECUSA a atualização enquanto houver operação em curso
// — ver `definirVerificadorDeOcupado()`.

#include <Arduino.h>
#include <functional>

namespace smi {
namespace dnc {
namespace esp {

class OtaUpdater {
public:
    // `nomeNaRede` é o que aparece na lista de portas do Arduino IDE.
    // `senha` protege a gravação; vazia é recusado de propósito.
    bool begin(const String& nomeNaRede, const String& senha);

    // Chamar em laço na tarefa de rede. Não bloqueia.
    void laco();

    bool iniciado() const { return m_iniciado; }
    bool atualizando() const { return m_atualizando; }

    // Deve devolver true quando houver transmissão em curso. Enquanto isso, a
    // atualização é recusada.
    void definirVerificadorDeOcupado(std::function<bool()> f) { m_ocupado = std::move(f); }

    // Para registrar no log do firmware.
    void definirLogger(std::function<void(const String&)> f) { m_log = std::move(f); }

private:
    bool m_iniciado = false;
    bool m_atualizando = false;
    std::function<bool()> m_ocupado;
    std::function<void(const String&)> m_log;
};

} // namespace esp
} // namespace dnc
} // namespace smi
