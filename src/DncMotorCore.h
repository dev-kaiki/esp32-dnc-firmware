#pragma once

// ---------------------------------------------------------------------------
// Motor de envio DNC — núcleo em C++ puro
// ---------------------------------------------------------------------------
//
// O QUE É
//
// A lógica que decide QUANTO e QUANDO escrever na CNC. É o que separa "mandar
// um arquivo pela serial" de um DNC: uma máquina que recebe rápido demais
// transborda o buffer e perde bloco no meio da usinagem.
//
// PORTE FIEL, NÃO REESCRITA
//
// Isto é o porte do motor que roda no aplicativo Android (`core/dnc/`), que já
// está validado em máquina real. A aritmética foi mantida byte a byte de
// propósito — inclusive as partes que parecem estranhas, porque elas foram
// calibradas com a CNC ligada, não deduzidas. Os testes em
// `ferramentas/testes/` fixam esses valores; se um deles quebrar, é porque
// alguém mudou comportamento, não estilo.
//
// SEM ARDUINO, SEM Qt
//
// Só <cstdint>, <algorithm> e <cstddef>. Isso é o que permite rodar os testes
// no PC, em segundos, sem gravar no ESP32 a cada mudança — o mesmo arranjo do
// DncProtocolCore. O tempo entra por parâmetro (`agoraMs`), nunca lido de
// dentro: é o que torna possível testar uma drenagem de 3 segundos sem esperar
// 3 segundos.

#include <cstdint>
#include <cstddef>

namespace smi {
namespace dnc {
namespace motor {

// --- Configuração -----------------------------------------------------------
//
// Só os campos que o motor realmente usa. O MachineConfig do Android tem
// dezenas de outros (caminhos, formatação de arquivo, rede) que não têm nada a
// dizer sobre o ritmo de envio.

enum class Paridade : uint8_t { Nenhuma = 0, Par = 1, Impar = 2 };

// Quem executa o controle de fluxo.
//
// A distinção importa e já custou caro: quando o CHIP faz XON/XOFF, o software
// nunca vê o XOFF passar, e qualquer lógica que dependa de enxergá-lo fica
// cega. Ver `modoDeFluxoManual()`.
enum class ControleDeFluxo : uint8_t {
    Nenhum   = 0,
    Software = 1,   // XON/XOFF executado pelo próprio chip/driver
    Hardware = 2,   // RTS/CTS executado pelo próprio chip/driver
};

struct ConfigDoMotor {
    int32_t  baudRate        = 9600;
    uint8_t  bitsDeDados     = 8;
    Paridade paridade        = Paridade::Nenhuma;
    uint8_t  bitsDeParada    = 1;

    ControleDeFluxo controleDeFluxo = ControleDeFluxo::Nenhum;

    // Fluxo tratado por NÓS, em vez de pelo chip.
    bool interpretarXonXoff      = false;
    bool exigirCtsAltoParaEnviar = false;
    bool exigirXonParaEnviar     = false;

    int limiteDoBufferDeEscrita = 256;

    // 0 = ritmo automático. Acima de zero, teto manual em bytes por segundo.
    int limiteManualDeTaxaBps = 0;

    int pedacoInicialBytes = 32;
    int pedacoMinimoBytes  = 8;
    int pedacoMaximoBytes  = 96;

    // Tempo sem a máquina consumir nada antes de segurar o envio.
    int esperaSemConsumoMs = 250;

    // Carência depois de uma liberação, para não oscilar em cima do limite.
    int debounceDeRetomadaMs = 40;

    // Prazo base da drenagem final.
    int drenagemTimeoutMs = 3000;
};

// Bits por caractere no fio: partida + dados + paridade + parada.
int bitsPorQuadro(const ConfigDoMotor& c);

// Quantos bytes por segundo o fio comporta nesta configuração.
int bytesPorSegundoNoFio(const ConfigDoMotor& c);

// True quando SOMOS NÓS que tratamos o fluxo, e não o chip.
//
// POR QUE ISTO EXISTE, com nome e sobrenome: com "XON/XOFF nativo do chip" o
// driver engole o XOFF antes de nós, então o motor nunca fica sabendo que a
// máquina mandou parar. O envio continua liberado, a fila não anda, e o que
// aparece na tela é um travamento sem causa. Nos modos manuais o motor enxerga
// os sinais e pode reagir — e é por isso que o ritmo é calculado diferente.
bool modoDeFluxoManual(const ConfigDoMotor& c);

// True quando há teto manual de taxa configurado.
bool modoDeTaxaEstrita(const ConfigDoMotor& c);

// --- Agendador de envio (TxScheduler) ---------------------------------------
//
// Decide o tamanho do próximo pedaço a escrever. Cresce enquanto a máquina
// acompanha, encolhe assim que ela reclama.
//
// A ASSIMETRIA É PROPOSITAL: encolher é rápido (metade de uma vez), crescer é
// lento (passos de 1 a 3 bytes, e só depois de progresso confirmado). Errar
// para menos custa velocidade; errar para mais custa uma peça.
class AgendadorDeEnvio {
public:
    void definirConfig(const ConfigDoMotor& c);

    // Tamanho do próximo pedaço, ou 0 quando não se deve escrever agora.
    // `bytesNaFila` é o que já está escrito e ainda não saiu pelo fio.
    int proximoPedaco(int64_t bytesRestantes, int64_t bytesNaFila) const;

    void aoEntrarEmEspera();      // a máquina mandou parar
    void aoAbrirJanela();         // liberou de novo
    void aoAceitarEscrita(int64_t bytes);
    void aoProgredirEstavel();    // andou sem reclamar

    void reiniciar();

    int     pedacoAtual() const        { return m_pedacoAtual; }
    int     menorPedacoVisto() const   { return m_menorPedacoVisto; }
    int     maiorPedacoVisto() const   { return m_maiorPedacoVisto; }
    int     subidas() const            { return m_subidas; }
    int     descidas() const           { return m_descidas; }
    int64_t bytesDesdeAJanela() const  { return m_bytesDesdeAJanela; }

private:
    void anotarObservacao();

    ConfigDoMotor m_config;
    int     m_pedacoAtual        = 32;
    int     m_menorPedacoVisto   = 0;
    int     m_maiorPedacoVisto   = 0;
    int     m_subidas            = 0;
    int     m_descidas           = 0;
    int     m_contadorEstavel    = 0;
    int64_t m_bytesDesdeAJanela  = 0;
    int64_t m_bytesEstaveis      = 0;
};

// --- Motivo de uma parada ---------------------------------------------------

enum class MotivoDeEspera : uint8_t {
    Nenhum = 0,
    PausaDoOperador,
    Xoff,
    CtsBaixo,
    SemConsumo,     // a máquina parou de puxar, sem dizer nada
};

// --- Controlador de fluxo (FlowController) ----------------------------------
//
// Reúne todas as razões para não transmitir num único "pode ou não pode".
// A ORDEM DA DECISÃO É A PRIORIDADE: pausa do operador vence tudo, depois
// XOFF, depois CTS, e por último a suspeita de máquina parada.
class ControladorDeFluxo {
public:
    void definirConfig(const ConfigDoMotor& c);

    void definirPausaDoOperador(bool pausado);
    void aoReceberXon();
    void aoReceberXoff();
    void aoMudarCts(bool alto);
    void definirEsperaDoVigia(bool esperar);

    void reiniciar();

    bool           envioPermitido() const { return m_motivo == MotivoDeEspera::Nenhum; }
    MotivoDeEspera motivoAtual() const    { return m_motivo; }

    // True quando a última chamada mudou a permissão. Substitui o sinal Qt do
    // lado Android: aqui quem chama consulta, em vez de ser avisado.
    bool mudouNaUltimaChamada() const { return m_mudouAgora; }

private:
    void recalcular();

    ConfigDoMotor m_config;
    bool m_pausadoPeloOperador = false;
    bool m_xoffAtivo           = false;
    bool m_ctsAlto             = true;
    bool m_esperaDoVigia       = false;
    MotivoDeEspera m_motivo    = MotivoDeEspera::Nenhum;
    bool m_ultimoPermitido     = false;
    bool m_mudouAgora          = false;
};

// --- Marca de tempo ---------------------------------------------------------
//
// Equivale ao QElapsedTimer do lado Android, mas SEM ler o relógio por dentro.
//
// Essa diferença é o que torna o motor testável: uma drenagem de 3 segundos se
// exercita em microssegundos, passando o tempo como número. Do outro lado, cada
// caso de drenagem custaria 3 segundos de espera real, e ninguém escreveria
// quarenta deles.
class Marca {
public:
    void iniciar(int64_t agoraMs) { m_marca = agoraMs; m_valida = true; }
    void invalidar()              { m_valida = false; m_marca = 0; }
    bool valida() const           { return m_valida; }
    int64_t decorrido(int64_t agoraMs) const { return m_valida ? (agoraMs - m_marca) : 0; }

private:
    int64_t m_marca = 0;
    bool    m_valida = false;
};

// --- Vigia de consumo (Watchdog) --------------------------------------------
//
// Detecta a máquina que parou de puxar bytes sem avisar nada — sem XOFF, sem
// baixar CTS. Acontece, e sem este vigia o motor continuaria empurrando contra
// um buffer parado até transbordar.
class VigiaDeConsumo {
public:
    void definirConfig(const ConfigDoMotor& c) { m_config = c; }

    void iniciar(int64_t agoraMs);
    void parar();

    // A máquina consumiu alguma coisa: o relógio de inatividade zera e uma
    // espera em curso é liberada.
    void anotarProgresso(int64_t agoraMs);

    // Libera a espera sem que tenha havido progresso (troca de estado, etc).
    void liberarEspera(int64_t agoraMs);

    // Chamar periodicamente. É aqui que a espera nasce.
    void verificar(int64_t agoraMs);

    bool emEspera() const             { return m_espera; }
    bool mudouNaUltimaChamada() const { return m_mudouAgora; }

private:
    ConfigDoMotor m_config;
    Marca m_progresso;
    bool  m_ativo      = false;
    bool  m_espera     = false;
    bool  m_mudouAgora = false;
};

// --- Controlador de drenagem (DrainController) ------------------------------
//
// A DRENAGEM É O FIM DO ENVIO, E É A PARTE QUE MAIS ENGANA.
//
// Quando o último byte sai da nossa fila, o programa NÃO chegou na máquina: ele
// ainda está atravessando o fio, o conversor e o buffer da CNC. Encerrar a
// sessão nesse instante corta o final do programa — e o operador só descobre
// quando a peça sai errada.
//
// Este controlador decide duas coisas opostas com cuidado:
//   - `podeConcluir()`  — já é seguro encerrar;
//   - `estourouPrazo()` — a drenagem empacou e não vai terminar sozinha.
//
// Os prazos são adaptativos porque um programa de 200 KB a 4800 baud demora
// minutos legítimos, enquanto 2 KB a 115200 baud tem que acabar em instantes.
// Um prazo fixo estaria errado nos dois casos.
class ControladorDeDrenagem {
public:
    void definirConfig(const ConfigDoMotor& c) { m_config = c; }

    void iniciar(int64_t agoraMs, int64_t bytesPendentes, double bpsEfetivo, double bpsDoFio);

    // Reabre o prazo sem recalcular os limiares.
    //
    // Usado quando o operador responde "continuar aguardando" ao aviso de
    // drenagem travada: o orçamento já estourou, e sem reabri-lo o motor ficaria
    // em estado de estouro para sempre.
    void reabrirPrazo(int64_t agoraMs);

    void reiniciar();

    bool ativo() const { return m_ativo; }

    // NÃO são const de propósito: as duas atualizam o histórico de progresso.
    // O original disfarça isso com const_cast; aqui a assinatura diz a verdade.
    bool estourouPrazo(int64_t agoraMs, int64_t bytesNaFila, bool envioPermitido);
    bool podeConcluir(int64_t agoraMs, int64_t bytesNaFila, bool envioPermitido);

    int64_t prazoMs() const               { return m_prazoMs; }
    int64_t limiarDeConclusaoBytes() const{ return m_limiarBytes; }
    int64_t janelaDeSilencioMs() const    { return m_janelaSilencioMs; }

    // Expostas para teste: são elas que carregam a aritmética calibrada.
    int64_t calcularPrazoMs(int64_t bytesPendentes, double bpsEfetivo, double bpsDoFio) const;
    int64_t calcularLimiarBytes(int64_t bytesPendentes) const;
    int64_t calcularJanelaSilencioMs(double bpsEfetivo, double bpsDoFio) const;

private:
    void zerarJanelaEstavel();
    void anotarProgressoDeDrenagem(int64_t agoraMs, int64_t bytesNaFila);

    ConfigDoMotor m_config;
    Marca m_relogio;      // desde o início da drenagem
    Marca m_estavel;      // desde que ficou parado perto do fim
    Marca m_progresso;    // desde o último byte que saiu

    bool    m_ativo            = false;
    int64_t m_prazoMs          = 0;
    int64_t m_limiarBytes      = 0;
    int64_t m_janelaSilencioMs = 0;
    int64_t m_ultimoBytesNaFila = -1;
    int64_t m_menorBytesNaFila  = -1;
};

// --- Motor de sessão de envio ------------------------------------------------
//
// O QUE FALTAVA NO ESP32
//
// O agendador sabe QUANTO escrever e o controlador de fluxo sabe SE pode. O que
// nao existia era quem soubesse ENCERRAR: quando o ultimo byte sai da fila, o
// programa ainda esta atravessando o fio e o buffer da CNC. Fechar ali corta o
// final do programa, e o operador so descobre na peca.
//
// Este e o porte dessa parte do DncEngine do aplicativo. NAO e o DncEngine
// inteiro, e a diferenca esta documentada: ficam de fora a recepcao (o firmware
// ja tem a sua), o ritmo especifico de USB (existe por causa da camada USB do
// tablet, que aqui nao existe) e o historico/rastreamento (armazenamento do
// Android).
//
// COMO SE USA: chamar `passo()` em laco. Ele devolve quantos bytes escrever
// agora -- zero quando nao se deve escrever. Depois de escrever de verdade,
// chamar `aoEscrever()` com o que a porta ACEITOU, nunca com o que se tentou.

enum class EstadoDaSessao : uint8_t {
    Parada = 0,
    AguardandoMaquina,
    Enviando,
    EsperaOperador,
    EsperaXoff,
    EsperaCts,
    EsperaSemConsumo,
    Drenando,
    AguardandoOperadorFinalizar,
    Concluida,
    Abortada,
};

// Avisos que a sessao produz para quem estiver olhando (log, portal, app).
// Sao consumidos um a um; nenhum se perde por falta de leitura imediata.
enum class AvisoDaSessao : uint8_t {
    Nenhum = 0,
    DrenagemIniciada,

    // A DRENAGEM EMPACOU -- E ISTO NAO ENCERRA NADA.
    //
    // A porta continua ABERTA. Quem decide e o operador: a maquina pode estar
    // apenas processando. Encerrar sozinho aqui foi o defeito que deixou
    // programa pela metade na maquina.
    DrenagemTravada,

    // A drenagem voltou a andar: a pergunta na tela nao vale mais.
    AvisoDeDrenagemLimpo,

    // Tudo entregue a porta. Ainda NAO se fecha: espera o operador confirmar
    // que a maquina terminou de executar.
    ProntoParaFinalizar,
};

// Por que a sessao terminou.
enum class FimDaSessao : uint8_t {
    Nenhum = 0,
    Concluida,          // operador confirmou o fim
    AbortadaPeloOperador,
};

class MotorDeSessao {
public:
    void definirConfig(const ConfigDoMotor& c);

    // `totalDoArquivo` e o tamanho ja formatado, em bytes.
    void iniciarEnvio(int64_t agoraMs, int64_t totalDoArquivo);

    void abortar(int64_t agoraMs);

    // Operador confirmou que a maquina terminou: pode fechar.
    void finalizar(int64_t agoraMs);

    // Operador respondeu "continuar aguardando" ao aviso de drenagem travada.
    // Reabre o prazo -- sem isto o motor ficaria em estado de estouro e a
    // pergunta voltaria no tique seguinte, para sempre.
    void continuarAguardando(int64_t agoraMs);

    void definirPausaDoOperador(bool pausado);
    void aoReceberXon();
    void aoReceberXoff();
    void aoMudarCts(bool alto);

    // Um passo do laco. Devolve quantos bytes escrever AGORA (0 = nenhum).
    //   `bytesRestantes`  o que ainda falta enviar (fila + arquivo)
    //   `bytesNaSaida`    o que ja foi escrito e ainda nao saiu pelo fio
    int passo(int64_t agoraMs, int64_t bytesRestantes, int64_t bytesNaSaida);

    // Chamar com o que a porta ACEITOU de verdade.
    void aoEscrever(int64_t agoraMs, int64_t bytesAceitos);

    EstadoDaSessao estado() const { return m_estado; }
    bool           ativa() const;
    FimDaSessao    fim() const { return m_fim; }

    // Retira o proximo aviso da fila, ou Nenhum se nao houver.
    AvisoDaSessao consumirAviso();

    int64_t bytesEntregues() const   { return m_entregues; }
    int64_t totalDoArquivo() const   { return m_total; }
    int64_t bytesPendentes() const   { return m_pendentesNoAviso; }

    MotivoDeEspera motivoDaEspera() const { return m_fluxo.motivoAtual(); }

    // Expostos para inspecao e teste.
    const AgendadorDeEnvio&      agendador() const { return m_agendador; }
    const ControladorDeDrenagem& drenagem() const  { return m_drenagem; }

private:
    void empilharAviso(AvisoDaSessao a);
    void atualizarEstadoDeEspera();
    void iniciarDrenagem(int64_t agoraMs, int64_t bytesNaSaida);

    // O vigia de consumo so vale sem controle de fluxo nenhum.
    //
    // REGRA HERDADA DO ORIGINAL, e ela e essencial: em XON/XOFF ou RTS/CTS,
    // ficar sem progresso NAO significa maquina travada -- significa que ela
    // mandou esperar. Ligar o vigia nesses modos gera espera falsa logo na
    // partida, e pior: como sem permissao nao se escreve, e sem escrever nao ha
    // progresso, a espera nunca se soltaria e o envio pararia no meio.
    bool usarVigiaDeConsumo() const;

    ConfigDoMotor        m_config;
    AgendadorDeEnvio     m_agendador;
    ControladorDeFluxo   m_fluxo;
    VigiaDeConsumo       m_vigia;
    ControladorDeDrenagem m_drenagem;

    EstadoDaSessao m_estado = EstadoDaSessao::Parada;
    FimDaSessao    m_fim    = FimDaSessao::Nenhum;

    bool    m_iniciada          = false;
    bool    m_avisoDeDrenagem   = false;
    bool    m_aguardandoFinal   = false;
    int64_t m_ultimaSaida       = -1;
    int64_t m_total             = 0;
    int64_t m_entregues         = 0;
    int64_t m_pendentesNoAviso  = 0;

    // Fila curta de avisos. Quatro basta: os avisos sao raros e quem consome
    // roda no mesmo laco.
    AvisoDaSessao m_avisos[4] = {AvisoDaSessao::Nenhum, AvisoDaSessao::Nenhum,
                                 AvisoDaSessao::Nenhum, AvisoDaSessao::Nenhum};
    uint8_t m_avisosInicio = 0;
    uint8_t m_avisosFim    = 0;
};

} // namespace motor
} // namespace dnc
} // namespace smi
