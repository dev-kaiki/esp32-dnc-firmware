#include "DncMotorCore.h"

#include <algorithm>

namespace smi {
namespace dnc {
namespace motor {

namespace {

// std::clamp existe em C++17, mas deixar explícito aqui evita depender da
// ordem de includes do toolchain do Arduino.
inline int limitar(int v, int minimo, int maximo) {
    return v < minimo ? minimo : (v > maximo ? maximo : v);
}

} // namespace

int bitsPorQuadro(const ConfigDoMotor& c) {
    const int bitsDeParidade = (c.paridade == Paridade::Nenhuma) ? 0 : 1;
    // Exatamente 2, não ">= 2": o original conta dois bits de parada só no caso
    // TwoStop. O valor 3 (um e meio, do QSerialPort) conta como um.
    const int parada = (c.bitsDeParada == 2) ? 2 : 1;
    return 1 + static_cast<int>(c.bitsDeDados) + bitsDeParidade + parada;
}

int bytesPorSegundoNoFio(const ConfigDoMotor& c) {
    const int quadro = std::max(1, bitsPorQuadro(c));
    return std::max(1, static_cast<int>(c.baudRate / quadro));
}

bool modoDeFluxoManual(const ConfigDoMotor& c) {
    return (c.controleDeFluxo != ControleDeFluxo::Software && c.interpretarXonXoff)
        || (c.controleDeFluxo != ControleDeFluxo::Hardware && c.exigirCtsAltoParaEnviar);
}

bool modoDeTaxaEstrita(const ConfigDoMotor& c) {
    return c.limiteManualDeTaxaBps > 0;
}

namespace {

// Os quatro tetos abaixo vieram calibrados da bancada com a CNC ligada. Os
// números não são redondos porque não foram escolhidos por gosto.

// Pedaço inicial no modo manual: o que cabe em 10 ms de fio.
int tetoInicialManual(const ConfigDoMotor& c) {
    const int porDezMs = std::max(1, (bytesPorSegundoNoFio(c) * 10) / 1000);
    return limitar(porDezMs, 12, 48);
}

// Teto de crescimento no modo manual: o que cabe em 60 ms de fio.
int tetoMaximoManual(const ConfigDoMotor& c) {
    const int porSessentaMs = std::max(1, (bytesPorSegundoNoFio(c) * 60) / 1000);
    const int tetoConfigurado = std::max(64, std::min(c.limiteDoBufferDeEscrita, 512));
    return limitar(std::max(porSessentaMs, tetoConfigurado / 2), 64, tetoConfigurado);
}

// Teto quando há limite manual de taxa: o que cabe em 12 ms do limite pedido,
// nunca mais que um sexto do que o fio entrega em um segundo.
int tetoTaxaEstrita(const ConfigDoMotor& c) {
    if (c.limiteManualDeTaxaBps <= 0) return 0;
    const int porTempo = std::max(8, (c.limiteManualDeTaxaBps * 12) / 1000);
    const int tetoDoFio = std::max(16, bytesPorSegundoNoFio(c) / 6);
    return limitar(porTempo, 8, std::min(128, tetoDoFio));
}

} // namespace

// --- AgendadorDeEnvio -------------------------------------------------------

void AgendadorDeEnvio::definirConfig(const ConfigDoMotor& c) {
    m_config = c;
    reiniciar();
}

void AgendadorDeEnvio::anotarObservacao() {
    if (m_menorPedacoVisto == 0 || m_pedacoAtual < m_menorPedacoVisto) m_menorPedacoVisto = m_pedacoAtual;
    if (m_pedacoAtual > m_maiorPedacoVisto) m_maiorPedacoVisto = m_pedacoAtual;
}

void AgendadorDeEnvio::reiniciar() {
    m_contadorEstavel   = 0;
    m_bytesDesdeAJanela = 0;
    m_bytesEstaveis     = 0;
    m_subidas           = 0;
    m_descidas          = 0;

    if (modoDeTaxaEstrita(m_config)) {
        m_pedacoAtual = std::min(16, tetoTaxaEstrita(m_config));
    } else if (modoDeFluxoManual(m_config)) {
        m_pedacoAtual = tetoInicialManual(m_config);
    } else {
        m_pedacoAtual = limitar(m_config.pedacoInicialBytes,
                                m_config.pedacoMinimoBytes,
                                m_config.pedacoMaximoBytes);
    }

    m_menorPedacoVisto = m_pedacoAtual;
    m_maiorPedacoVisto = m_pedacoAtual;
}

void AgendadorDeEnvio::aoEntrarEmEspera() {
    m_contadorEstavel = 0;
    const int antes = m_pedacoAtual;

    if (modoDeTaxaEstrita(m_config)) {
        m_pedacoAtual = std::max(1, m_pedacoAtual / 2);
    } else if (modoDeFluxoManual(m_config)) {
        // A dose da redução depende de quanto se conseguiu enviar desde a
        // última liberação. Pouco entregue significa que erramos feio no
        // tamanho; muito entregue significa que quase acertamos.
        const int piso = std::max(4, tetoInicialManual(m_config));
        if (m_bytesDesdeAJanela < (antes / 2)) {
            m_pedacoAtual = std::max(piso, m_pedacoAtual / 2);
        } else if (m_bytesDesdeAJanela < (antes * 2)) {
            m_pedacoAtual = std::max(piso, (m_pedacoAtual * 2) / 3);
        } else {
            m_pedacoAtual = std::max(piso, (m_pedacoAtual * 3) / 4);
        }
    } else {
        m_pedacoAtual = std::max(m_config.pedacoMinimoBytes, m_pedacoAtual / 2);
    }

    if (m_pedacoAtual < antes) ++m_descidas;
    m_bytesDesdeAJanela = 0;
    m_bytesEstaveis     = 0;
    anotarObservacao();
}

void AgendadorDeEnvio::aoAbrirJanela() {
    m_contadorEstavel   = 0;
    m_bytesDesdeAJanela = 0;
    m_bytesEstaveis     = 0;
}

void AgendadorDeEnvio::aoAceitarEscrita(int64_t bytes) {
    if (bytes > 0) {
        m_bytesDesdeAJanela += bytes;
        m_bytesEstaveis     += bytes;
    }
}

void AgendadorDeEnvio::aoProgredirEstavel() {
    const int antes = m_pedacoAtual;

    if (modoDeTaxaEstrita(m_config)) {
        ++m_contadorEstavel;
        const int teto = tetoTaxaEstrita(m_config);
        m_pedacoAtual = std::min(teto, m_pedacoAtual + std::max(2, teto / 8));
    } else if (modoDeFluxoManual(m_config)) {
        // Só cresce depois de ter entregue o equivalente a três pedaços sem
        // reclamação. Sem essa exigência, uma única janela boa faria o motor
        // subir e a máquina transbordar na janela seguinte.
        const int64_t limiarDeCrescimento =
            std::max<int64_t>(8, static_cast<int64_t>(m_pedacoAtual) * 3);
        if (m_bytesEstaveis < limiarDeCrescimento) {
            anotarObservacao();
            return;
        }

        ++m_contadorEstavel;
        if (m_contadorEstavel >= 2) {
            const int teto = tetoMaximoManual(m_config);
            const int passo = (m_pedacoAtual < 12) ? 1 : ((m_pedacoAtual < 24) ? 2 : 3);
            m_pedacoAtual = std::min(teto, m_pedacoAtual + passo);
            m_contadorEstavel = 0;
            m_bytesEstaveis   = 0;
        }
    } else {
        ++m_contadorEstavel;
        m_pedacoAtual = std::min(m_config.pedacoMaximoBytes,
                                 m_pedacoAtual + std::max(1, m_config.pedacoMinimoBytes / 2));
    }

    if (m_pedacoAtual > antes) ++m_subidas;
    anotarObservacao();
}

int AgendadorDeEnvio::proximoPedaco(int64_t bytesRestantes, int64_t bytesNaFila) const {
    if (bytesRestantes <= 0) return 0;

    if (modoDeTaxaEstrita(m_config)) {
        const int tetoDeMarca = std::max(tetoTaxaEstrita(m_config), m_pedacoAtual * 2);
        const int marcaDagua = std::min(std::max(16, m_config.limiteDoBufferDeEscrita), tetoDeMarca);
        const int margemLivre = std::max(0, marcaDagua - static_cast<int>(bytesNaFila));
        if (margemLivre <= 0) return 0;
        const int desejado = std::min(m_pedacoAtual, margemLivre);
        const int teto = tetoTaxaEstrita(m_config);
        const int limitado = std::max(1, std::min(desejado, teto));
        return static_cast<int>(std::min<int64_t>(limitado, bytesRestantes));
    }

    if (modoDeFluxoManual(m_config)) {
        const int marcaDagua = limitar(
            std::max(m_pedacoAtual * 3, tetoInicialManual(m_config) * 5),
            48,
            std::max(64, std::min(m_config.limiteDoBufferDeEscrita, 512)));
        const int margemLivre = std::max(0, marcaDagua - static_cast<int>(bytesNaFila));
        if (margemLivre <= 0) return 0;
        const int desejado = std::min(m_pedacoAtual, margemLivre);
        const int limitado = std::max(1, std::min(desejado, tetoMaximoManual(m_config)));
        return static_cast<int>(std::min<int64_t>(limitado, bytesRestantes));
    }

    const int margemLivre = std::max(0, m_config.limiteDoBufferDeEscrita - static_cast<int>(bytesNaFila));
    if (margemLivre <= 0) return 0;
    const int desejado = std::min(m_pedacoAtual, margemLivre);
    const int limitado = std::max(m_config.pedacoMinimoBytes,
                                  std::min(desejado, m_config.pedacoMaximoBytes));
    return static_cast<int>(std::min<int64_t>(limitado, bytesRestantes));
}

// --- ControladorDeFluxo -----------------------------------------------------

void ControladorDeFluxo::definirConfig(const ConfigDoMotor& c) {
    m_config = c;
    reiniciar();
}

void ControladorDeFluxo::reiniciar() {
    m_pausadoPeloOperador = false;

    // COMEÇA LIBERADO, a menos que se exija XON explicitamente.
    //
    // Muitas CNCs em XON/XOFF não mandam um XON inicial ao abrir a sessão.
    // Se o motor começasse esperando por ele, essas máquinas nunca receberiam
    // o primeiro byte — e o sintoma seria "não envia", sem nenhuma pista.
    m_xoffAtivo = m_config.exigirXonParaEnviar;
    m_ctsAlto   = !m_config.exigirCtsAltoParaEnviar;

    m_esperaDoVigia   = false;
    m_motivo          = MotivoDeEspera::Nenhum;
    m_ultimoPermitido = false;
    m_mudouAgora      = false;
    recalcular();
}

void ControladorDeFluxo::definirPausaDoOperador(bool pausado) { m_pausadoPeloOperador = pausado; recalcular(); }
void ControladorDeFluxo::aoReceberXon()                       { m_xoffAtivo = false;             recalcular(); }
void ControladorDeFluxo::aoReceberXoff()                      { m_xoffAtivo = true;              recalcular(); }
void ControladorDeFluxo::aoMudarCts(bool alto)                { m_ctsAlto = alto;                recalcular(); }
void ControladorDeFluxo::definirEsperaDoVigia(bool esperar)   { m_esperaDoVigia = esperar;       recalcular(); }

void ControladorDeFluxo::recalcular() {
    MotivoDeEspera novo = MotivoDeEspera::Nenhum;

    // A ordem é a prioridade, e ela importa para o que aparece na tela: se a
    // máquina mandou XOFF e o operador também pausou, o motivo mostrado deve
    // ser a pausa do operador — é a informação acionável.
    if (m_pausadoPeloOperador)                                novo = MotivoDeEspera::PausaDoOperador;
    else if (m_config.interpretarXonXoff && m_xoffAtivo)      novo = MotivoDeEspera::Xoff;
    else if (m_config.exigirCtsAltoParaEnviar && !m_ctsAlto)  novo = MotivoDeEspera::CtsBaixo;
    else if (m_esperaDoVigia)                                 novo = MotivoDeEspera::SemConsumo;

    const bool permitido = (novo == MotivoDeEspera::Nenhum);
    m_mudouAgora = (m_motivo != novo) || (m_ultimoPermitido != permitido);
    m_motivo = novo;
    m_ultimoPermitido = permitido;
}

// --- VigiaDeConsumo ---------------------------------------------------------

void VigiaDeConsumo::iniciar(int64_t agoraMs) {
    m_progresso.iniciar(agoraMs);
    m_espera = false;
    m_ativo = true;
    m_mudouAgora = false;
}

void VigiaDeConsumo::parar() {
    m_ativo = false;
    m_mudouAgora = m_espera;
    m_espera = false;
    m_progresso.invalidar();
}

void VigiaDeConsumo::anotarProgresso(int64_t agoraMs) {
    m_progresso.iniciar(agoraMs);
    m_mudouAgora = m_espera;
    m_espera = false;
}

void VigiaDeConsumo::liberarEspera(int64_t agoraMs) {
    m_mudouAgora = m_espera;
    m_espera = false;
    m_progresso.iniciar(agoraMs);
}

void VigiaDeConsumo::verificar(int64_t agoraMs) {
    m_mudouAgora = false;
    if (!m_ativo) return;

    // Zero desliga o vigia. Existe porque em máquina lenta de verdade a espera
    // dispararia o tempo todo e atrapalharia mais do que ajuda.
    if (m_config.esperaSemConsumoMs <= 0) return;

    if (!m_espera && m_progresso.valida()
        && m_progresso.decorrido(agoraMs) >= m_config.esperaSemConsumoMs) {
        m_espera = true;
        m_mudouAgora = true;
    }
}

// --- ControladorDeDrenagem --------------------------------------------------

namespace {

inline int64_t limitar64(int64_t v, int64_t minimo, int64_t maximo) {
    return v < minimo ? minimo : (v > maximo ? maximo : v);
}

// Teto arredondado para cima, sem depender de <cmath> no alvo embarcado.
inline int64_t tetoDivisao(double numerador, double denominador) {
    if (denominador <= 0.0) return 0;
    const double q = numerador / denominador;
    const int64_t inteiro = static_cast<int64_t>(q);
    return (q > static_cast<double>(inteiro)) ? inteiro + 1 : inteiro;
}

} // namespace

int64_t ControladorDeDrenagem::calcularPrazoMs(int64_t bytesPendentes, double bpsEfetivo, double bpsDoFio) const {
    const int64_t base = std::max<int64_t>(3000, m_config.drenagemTimeoutMs);

    // Taxa abaixo de 1 B/s é ruído de medição, não informação. Tratada como
    // ausente para não gerar prazos absurdos.
    const double efetivo = (bpsEfetivo > 1.0) ? bpsEfetivo : 0.0;
    const double fio     = (bpsDoFio  > 1.0) ? bpsDoFio  : 0.0;

    double referencia = 0.0;
    if (efetivo > 0.0 && fio > 0.0) {
        referencia = std::min(fio, std::max(16.0, efetivo));
    } else if (efetivo > 0.0) {
        referencia = std::max(16.0, efetivo);
    } else if (fio > 0.0) {
        // Sem medição própria, supõe-se um quarto do fio: a CNC quase nunca
        // consome na velocidade nominal.
        referencia = std::max(16.0, fio * 0.25);
    } else {
        referencia = 32.0;
    }

    const int64_t estimado = tetoDivisao(static_cast<double>(std::max<int64_t>(0, bytesPendentes)) * 1000.0, referencia);
    const int64_t folga = limitar64(1500 + (bytesPendentes * 2), 5000, 45000);
    return limitar64(std::max(base, estimado + folga), 8000, 180000);
}

int64_t ControladorDeDrenagem::calcularLimiarBytes(int64_t bytesPendentes) const {
    const int64_t pendente = std::max<int64_t>(0, bytesPendentes);
    if (pendente <= 0) return 0;
    const int64_t proporcional = pendente / 3;
    const int64_t protegido = limitar64(proporcional, 96, 768);
    return std::min(pendente, protegido);
}

int64_t ControladorDeDrenagem::calcularJanelaSilencioMs(double bpsEfetivo, double bpsDoFio) const {
    double referencia = 16.0;
    if (bpsEfetivo > referencia) referencia = bpsEfetivo;
    if ((bpsDoFio * 0.20) > referencia) referencia = bpsDoFio * 0.20;

    const int64_t adaptativa = tetoDivisao(9000.0, referencia);
    const int64_t comDebounce = std::max<int64_t>(adaptativa, static_cast<int64_t>(m_config.debounceDeRetomadaMs) * 6);
    return limitar64(comDebounce, 600, 2500);
}

void ControladorDeDrenagem::iniciar(int64_t agoraMs, int64_t bytesPendentes, double bpsEfetivo, double bpsDoFio) {
    m_ativo = true;
    m_prazoMs          = calcularPrazoMs(bytesPendentes, bpsEfetivo, bpsDoFio);
    m_limiarBytes      = calcularLimiarBytes(bytesPendentes);
    m_janelaSilencioMs = calcularJanelaSilencioMs(bpsEfetivo, bpsDoFio);
    m_ultimoBytesNaFila = bytesPendentes;
    m_menorBytesNaFila  = bytesPendentes;
    m_relogio.iniciar(agoraMs);
    m_progresso.iniciar(agoraMs);
    zerarJanelaEstavel();
}

void ControladorDeDrenagem::reabrirPrazo(int64_t agoraMs) {
    if (!m_ativo) return;
    m_relogio.iniciar(agoraMs);
    m_progresso.iniciar(agoraMs);
    zerarJanelaEstavel();
}

void ControladorDeDrenagem::reiniciar() {
    m_ativo = false;
    m_prazoMs = 0;
    m_limiarBytes = 0;
    m_janelaSilencioMs = 0;
    m_ultimoBytesNaFila = -1;
    m_menorBytesNaFila = -1;
    m_relogio.invalidar();
    m_progresso.invalidar();
    m_estavel.invalidar();
}

void ControladorDeDrenagem::zerarJanelaEstavel() {
    m_estavel.invalidar();
}

void ControladorDeDrenagem::anotarProgressoDeDrenagem(int64_t agoraMs, int64_t bytesNaFila) {
    if (!m_ativo) return;

    const bool progrediu = (m_ultimoBytesNaFila < 0 || bytesNaFila < m_ultimoBytesNaFila);
    if (!m_progresso.valida() || progrediu) {
        m_progresso.iniciar(agoraMs);
        zerarJanelaEstavel();
    }
    if (m_menorBytesNaFila < 0 || bytesNaFila < m_menorBytesNaFila) {
        m_menorBytesNaFila = bytesNaFila;
    }
    m_ultimoBytesNaFila = bytesNaFila;
}

bool ControladorDeDrenagem::estourouPrazo(int64_t agoraMs, int64_t bytesNaFila, bool envioPermitido) {
    if (!m_ativo || !m_relogio.valida()) return false;

    anotarProgressoDeDrenagem(agoraMs, bytesNaFila);

    // COM O ENVIO SEGURADO NÃO SE CONTA ESTOURO.
    //
    // A máquina mandou parar, ou o operador pausou. Chamar isso de "drenagem
    // travada" seria acusar a CNC de um problema que ela mesma criou de
    // propósito, e encerrar a sessão por causa disso.
    if (!envioPermitido) return false;

    if (podeConcluir(agoraMs, bytesNaFila, envioPermitido)) return false;

    const int64_t prazo = std::max<int64_t>(3000, m_prazoMs);

    if (bytesNaFila <= 0) {
        return m_relogio.decorrido(agoraMs) > prazo;
    }

    // Muita coisa parada há muito tempo: não adianta esperar o prazo inteiro.
    const int64_t residuoGrave = std::max<int64_t>(m_limiarBytes * 2, 1024);
    const int64_t prazoDeEmpaque = limitar64(prazo / 2, 5000, 45000);
    if (bytesNaFila > residuoGrave && m_progresso.valida()
        && m_progresso.decorrido(agoraMs) > prazoDeEmpaque) {
        return true;
    }

    return m_relogio.decorrido(agoraMs) > prazo;
}

bool ControladorDeDrenagem::podeConcluir(int64_t agoraMs, int64_t bytesNaFila, bool envioPermitido) {
    if (!m_ativo) return false;

    anotarProgressoDeDrenagem(agoraMs, bytesNaFila);

    if (!envioPermitido && bytesNaFila > 0) {
        zerarJanelaEstavel();
        return false;
    }

    const int64_t estavelMs = limitar64(
        std::max<int64_t>(m_janelaSilencioMs, static_cast<int64_t>(m_config.debounceDeRetomadaMs) * 4),
        180, 3000);

    // Fila vazia: ainda assim espera a janela de silêncio antes de encerrar.
    // Fila vazia significa "saiu de nós", não "chegou na máquina".
    if (bytesNaFila <= 0) {
        if (!m_estavel.valida()) {
            m_estavel.iniciar(agoraMs);
            return false;
        }
        return m_estavel.decorrido(agoraMs) >= estavelMs;
    }

    const bool quaseNoFim = (m_limiarBytes > 0 && bytesNaFila <= m_limiarBytes);
    const bool paradoHaAlgumTempo = m_progresso.valida() && m_progresso.decorrido(agoraMs) >= estavelMs;
    if (!quaseNoFim || !paradoHaAlgumTempo) {
        zerarJanelaEstavel();
        return false;
    }

    if (!m_estavel.valida()) {
        m_estavel.iniciar(agoraMs);
        return false;
    }

    return m_estavel.decorrido(agoraMs) >= estavelMs;
}

// --- MotorDeSessao ----------------------------------------------------------

bool MotorDeSessao::usarVigiaDeConsumo() const {
    // Sem fluxo nenhum, ficar parado e mesmo sinal de problema. Com fluxo,
    // ficar parado e a maquina mandando esperar. Ver a nota no cabecalho.
    return !modoDeFluxoManual(m_config)
        && m_config.controleDeFluxo == ControleDeFluxo::Nenhum;
}

void MotorDeSessao::definirConfig(const ConfigDoMotor& c) {
    m_config = c;
    m_agendador.definirConfig(c);
    m_fluxo.definirConfig(c);
    m_vigia.definirConfig(c);
    m_drenagem.definirConfig(c);
}

void MotorDeSessao::empilharAviso(AvisoDaSessao a) {
    const uint8_t proximo = (uint8_t)((m_avisosFim + 1) % 4);
    if (proximo == m_avisosInicio) return;   // fila cheia: descarta o mais novo
    m_avisos[m_avisosFim] = a;
    m_avisosFim = proximo;
}

AvisoDaSessao MotorDeSessao::consumirAviso() {
    if (m_avisosInicio == m_avisosFim) return AvisoDaSessao::Nenhum;
    const AvisoDaSessao a = m_avisos[m_avisosInicio];
    m_avisosInicio = (uint8_t)((m_avisosInicio + 1) % 4);
    return a;
}

bool MotorDeSessao::ativa() const {
    return m_iniciada
        && m_estado != EstadoDaSessao::Concluida
        && m_estado != EstadoDaSessao::Abortada
        && m_estado != EstadoDaSessao::Parada;
}

void MotorDeSessao::iniciarEnvio(int64_t agoraMs, int64_t totalDoArquivo) {
    m_agendador.reiniciar();
    m_fluxo.reiniciar();
    m_drenagem.reiniciar();
    m_vigia.iniciar(agoraMs);

    m_estado = EstadoDaSessao::AguardandoMaquina;
    m_fim = FimDaSessao::Nenhum;
    m_iniciada = true;
    m_avisoDeDrenagem = false;
    m_aguardandoFinal = false;
    m_ultimaSaida = -1;
    m_total = totalDoArquivo > 0 ? totalDoArquivo : 0;
    m_entregues = 0;
    m_pendentesNoAviso = 0;
    m_avisosInicio = m_avisosFim = 0;
}

void MotorDeSessao::abortar(int64_t /*agoraMs*/) {
    if (!m_iniciada) return;
    m_iniciada = false;
    m_vigia.parar();
    m_drenagem.reiniciar();
    m_estado = EstadoDaSessao::Abortada;
    m_fim = FimDaSessao::AbortadaPeloOperador;
}

void MotorDeSessao::finalizar(int64_t /*agoraMs*/) {
    if (!m_iniciada) return;
    m_iniciada = false;
    m_vigia.parar();
    m_drenagem.reiniciar();
    m_estado = EstadoDaSessao::Concluida;
    m_fim = FimDaSessao::Concluida;
}

void MotorDeSessao::continuarAguardando(int64_t agoraMs) {
    if (!m_iniciada || !m_avisoDeDrenagem) return;
    // REABRIR O PRAZO E OBRIGATORIO. Sem isso o orcamento continua estourado e
    // a pergunta volta no proximo passo, num laco que nunca sai do lugar.
    m_avisoDeDrenagem = false;
    m_drenagem.reabrirPrazo(agoraMs);
}

void MotorDeSessao::definirPausaDoOperador(bool pausado) { m_fluxo.definirPausaDoOperador(pausado); }
void MotorDeSessao::aoReceberXon()                       { m_fluxo.aoReceberXon(); }
void MotorDeSessao::aoReceberXoff()                      { m_fluxo.aoReceberXoff(); }
void MotorDeSessao::aoMudarCts(bool alto)                { m_fluxo.aoMudarCts(alto); }

void MotorDeSessao::atualizarEstadoDeEspera() {
    switch (m_fluxo.motivoAtual()) {
    case MotivoDeEspera::PausaDoOperador: m_estado = EstadoDaSessao::EsperaOperador;   break;
    case MotivoDeEspera::Xoff:            m_estado = EstadoDaSessao::EsperaXoff;       break;
    case MotivoDeEspera::CtsBaixo:        m_estado = EstadoDaSessao::EsperaCts;        break;
    case MotivoDeEspera::SemConsumo:      m_estado = EstadoDaSessao::EsperaSemConsumo; break;
    default:                              m_estado = EstadoDaSessao::Enviando;         break;
    }
}

void MotorDeSessao::iniciarDrenagem(int64_t agoraMs, int64_t bytesNaSaida) {
    if (m_drenagem.ativo()) return;

    // O VIGIA PARA AQUI, como no original. Durante a drenagem nao ha mais o que
    // escrever, entao "sem progresso local" deixa de ser sintoma de problema --
    // e o que resta e justamente esperar a maquina consumir.
    m_vigia.parar();
    m_fluxo.definirEsperaDoVigia(false);

    m_estado = EstadoDaSessao::Drenando;
    m_drenagem.iniciar(agoraMs, bytesNaSaida,
                       0.0,
                       (double)bytesPorSegundoNoFio(m_config));
    empilharAviso(AvisoDaSessao::DrenagemIniciada);
}

int MotorDeSessao::passo(int64_t agoraMs, int64_t bytesRestantes, int64_t bytesNaSaida) {
    if (!m_iniciada) return 0;
    if (m_estado == EstadoDaSessao::AguardandoOperadorFinalizar) return 0;

    // Vigia: so em modo sem fluxo, e nunca durante a drenagem.
    if (usarVigiaDeConsumo() && !m_drenagem.ativo()) {
        // PROGRESSO E BYTE SAINDO PELO FIO, NAO BYTE QUE NOS ESCREVEMOS.
        //
        // Escrever num buffer nao prova que a maquina consumiu. Se o vigia
        // dependesse so das nossas escritas, ele travaria justamente quando o
        // buffer enche: contrapressao normal viraria "maquina parada", a
        // permissao cairia, e sem permissao nunca mais escreveriamos -- o envio
        // pararia no meio do programa e nao voltaria sozinho.
        //
        // No aplicativo esse sinal vem do sistema avisando que a porta aceitou
        // bytes. Aqui, o equivalente e a fila de saida da UART diminuindo.
        if (m_ultimaSaida < 0 || bytesNaSaida < m_ultimaSaida) {
            m_vigia.anotarProgresso(agoraMs);
        }
        m_ultimaSaida = bytesNaSaida;

        m_vigia.verificar(agoraMs);
        m_fluxo.definirEsperaDoVigia(m_vigia.emEspera());
    }

    const bool permitido = m_fluxo.envioPermitido();

    // --- Drenagem -----------------------------------------------------------
    if (m_drenagem.ativo()) {
        if (m_drenagem.podeConcluir(agoraMs, bytesNaSaida, permitido)) {
            if (!m_aguardandoFinal) {
                m_aguardandoFinal = true;
                // A drenagem chegou ao fim depois de ter travado: a pergunta na
                // tela nao vale mais e precisa sumir, senao o operador responde
                // a uma situacao que ja passou.
                if (m_avisoDeDrenagem) {
                    m_avisoDeDrenagem = false;
                    empilharAviso(AvisoDaSessao::AvisoDeDrenagemLimpo);
                }
                m_estado = EstadoDaSessao::AguardandoOperadorFinalizar;
                empilharAviso(AvisoDaSessao::ProntoParaFinalizar);
            }
            return 0;
        }

        // NAO ENCERRA E NAO INTERROMPE: so pergunta.
        //
        // Este e o ponto exato onde a versao anterior fechava a porta sozinha.
        // Avisar e continuar deixa a maquina terminar se ela ainda estiver
        // processando -- e se voltar a andar, o aviso e retirado sozinho logo
        // abaixo.
        if (m_drenagem.estourouPrazo(agoraMs, bytesNaSaida, permitido)) {
            if (!m_avisoDeDrenagem) {
                m_avisoDeDrenagem = true;
                m_pendentesNoAviso = bytesNaSaida;
                empilharAviso(AvisoDaSessao::DrenagemTravada);
            }
        } else if (m_avisoDeDrenagem) {
            m_avisoDeDrenagem = false;
            empilharAviso(AvisoDaSessao::AvisoDeDrenagemLimpo);
        }
        return 0;
    }

    // --- Envio --------------------------------------------------------------
    if (!permitido) {
        if (m_estado == EstadoDaSessao::Enviando
            || m_estado == EstadoDaSessao::AguardandoMaquina) {
            m_agendador.aoEntrarEmEspera();
        }
        atualizarEstadoDeEspera();
        return 0;
    }

    if (m_estado != EstadoDaSessao::Enviando) {
        m_agendador.aoAbrirJanela();
        m_estado = EstadoDaSessao::Enviando;
    }

    // Nada mais a enviar: comeca a drenagem, que e o que garante que o final do
    // programa chega inteiro na maquina.
    if (bytesRestantes <= 0) {
        iniciarDrenagem(agoraMs, bytesNaSaida);
        return 0;
    }

    return m_agendador.proximoPedaco(bytesRestantes, bytesNaSaida);
}

void MotorDeSessao::aoEscrever(int64_t agoraMs, int64_t bytesAceitos) {
    if (!m_iniciada || bytesAceitos <= 0) return;

    m_entregues += bytesAceitos;
    m_agendador.aoAceitarEscrita(bytesAceitos);
    m_vigia.anotarProgresso(agoraMs);
    m_fluxo.definirEsperaDoVigia(m_vigia.emEspera());

    // Crescimento so com entrega relevante nos modos manuais.
    //
    // Herdado do original: em fluxo manual, crescer a cada devolucao de escrita
    // gera oscilacao -- sobe, a maquina reclama, desce, sobe de novo. O corte em
    // 16 bytes separa entrega de verdade de migalha.
    if (!modoDeFluxoManual(m_config) || bytesAceitos >= 16) {
        m_agendador.aoProgredirEstavel();
    }
}

} // namespace motor
} // namespace dnc
} // namespace smi
