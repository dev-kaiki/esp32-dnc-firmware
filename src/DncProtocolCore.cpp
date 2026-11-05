#include "DncProtocolCore.h"

#include <cstring>

namespace smi {
namespace dnc {
namespace esp {

namespace P = smi::dnc::proto;

const char* nomeDoEstado(uint8_t e)
{
    switch (e) {
    case EST_IDLE:                return "IDLE";
    case EST_RECEIVING_FILE:      return "RECEIVING_FILE";
    case EST_VALIDATING_FILE:     return "VALIDATING_FILE";
    case EST_READY:               return "READY";
    case EST_SENDING:             return "SENDING";
    case EST_ONLINE:              return "ONLINE";
    case EST_PAUSED:              return "PAUSED";
    case EST_RECEIVING_FROM_CNC:  return "RECEIVING_FROM_CNC";
    case EST_COMPLETED:           return "COMPLETED";
    case EST_CANCELLED:           return "CANCELLED";
    case EST_ERROR:               return "ERROR";
    case EST_AWAIT_FINISH:        return "AWAIT_FINISH";
    case EST_DRAIN_STALLED:       return "DRAIN_STALLED";
    default:                      return "?";
    }
}

DncProtocolCore::DncProtocolCore(IArmazenamento& armazenamento,
                                 ITransporte& transporte,
                                 IAcoesDnc& acoes)
    : m_arm(armazenamento), m_trans(transporte), m_acoes(acoes) {}

void DncProtocolCore::definirIdentidade(const std::string& firmware,
                                        const std::string& idAparelho,
                                        uint8_t linhasDisponiveis)
{
    m_firmware = firmware;
    m_idAparelho = idAparelho;

    // Só entram aqui as linhas REALMENTE ligadas na placa. O app mostra LED
    // apenas para essas — nunca simula um sinal que não está soldado, porque
    // LED que mente é pior que LED que falta.
    m_linhas = linhasDisponiveis;
}

// ---------------------------------------------------------------------------
// Entrada
// ---------------------------------------------------------------------------

void DncProtocolCore::aoReceberBytes(const uint8_t* dados, size_t n)
{
    m_decod.alimentar(dados, n);

    P::Quadro q;
    for (;;) {
        const auto r = m_decod.proximo(q);
        if (r == P::DECOD_QUADRO)      tratarQuadro(q);
        else if (r == P::DECOD_VAZIO)  break;
        else if (r == P::DECOD_ESTOURO) {
            notificarLog("Fluxo invalido do app; buffer reiniciado.");
            break;
        }
        // Lixo e CRC inválido: o decodificador já ressincronizou.
    }
}

void DncProtocolCore::aoPerderConexao()
{
    m_temControlador = false;
    m_idControlador.clear();

    // OS OPIDS VISTOS MORREM COM A CONEXAO.
    //
    // A lista existe para reconhecer a REPETICAO de um comando dentro da mesma
    // conversa (resposta perdida, app reenvia). Mantida entre conexoes, ela
    // fazia o app reaberto -- que recomecava a numerar do 1 -- ter o HELLO e
    // os comandos seguintes "reconhecidos como repetidos": ACK sem executar,
    // inclusive bloco de arquivo e configuracao. O app novo ja sorteia o
    // inicio; isto fecha o mesmo buraco do lado da caixa, para app antigo.
    m_dedup.limpar();

    // Download pela metade morre junto: nao ha para quem mandar o resto, e
    // deixar o arquivo aberto seguraria o cartao.
    encerrarEnvioDeArquivo(false);

    // Upload pela metade é descartado: o temporário morre e o definitivo nunca
    // existiu.
    if (m_estado == EST_RECEIVING_FILE) {
        m_arm.descartarTemporario();
        m_recebido = 0;
        definirEstado(EST_IDLE);
    }

    // TRANSMISSÃO EM ANDAMENTO NÃO É CANCELADA.
    //
    // Se a CNC está sendo alimentada e o Wi-Fi cai, abortar deixaria a máquina
    // com o programa pela metade — que é exatamente o estrago que este projeto
    // passou o dia inteiro tentando evitar. A caixa termina o trabalho
    // sozinha; o app reconecta e reencontra a operação em curso.
    //
    // O risco oposto — a caixa continuar alimentando sem ninguém olhando — é
    // menor: o controle de fluxo da própria máquina continua valendo, e parar
    // no meio é pior que terminar.
}

// ---------------------------------------------------------------------------
// Máquina de estados
// ---------------------------------------------------------------------------

bool DncProtocolCore::foraDeOperacao() const
{
    // ERRO NAO PODE SER BECO SEM SAIDA.
    //
    // EST_ERROR nao estava em lista nenhuma: nem para configurar, nem para
    // subir arquivo, nem para iniciar, nem sequer para cancelar. Uma falha
    // qualquer deixava a caixa recusando TUDO ate alguem tirar da tomada -- e a
    // unica coisa que o operador via era "comando invalido nesse estado", que
    // nao diz o que fazer.
    //
    // E o estado de erro e, por definicao, um estado SEM operacao em curso:
    // recomecar do zero e justamente a recuperacao correta, nao um risco. Nada
    // esta sendo transmitido para a maquina que possa ser atropelado.
    //
    // AWAIT_FINISH e DRAIN_STALLED continuam de fora, e isso e deliberado: la a
    // porta esta ABERTA e a maquina pode estar executando. Quem decide encerrar
    // e o operador, com OP_FINISH ou OP_CANCEL.
    return m_estado == EST_IDLE
        || m_estado == EST_READY
        || m_estado == EST_COMPLETED
        || m_estado == EST_CANCELLED
        || m_estado == EST_ERROR;
}

bool DncProtocolCore::comandoCabeNoEstado(uint8_t tipo) const
{
    switch (tipo) {
    // Sempre aceitos: são sobre a sessão, não sobre a máquina.
    case P::TIPO_HELLO:
    case P::TIPO_PING:
    case P::TIPO_STATE_QUERY:
    case P::TIPO_CLAIM:
    case P::TIPO_RELEASE:
        return true;

    // Configuração NUNCA durante transmissão.
    //
    // Trocar baud com a máquina recebendo corromperia o resto do programa —
    // e o operador só descobriria na peça.
    case P::TIPO_CONFIG_SET:
        return foraDeOperacao();

    // Varrer o ar e gravar rede derrubam a conexao e param o radio por
    // segundos. Com a maquina sendo alimentada isso cortaria o programa no
    // meio, entao vale a mesma regra da configuracao serial.
    case P::TIPO_WIFI_SCAN:
    case P::TIPO_WIFI_SET:
        return foraDeOperacao();

    case P::TIPO_FILE_BEGIN:
        // Sobrescrever o programa em uso é proibido: só fora de operação.
        return foraDeOperacao();

    case P::TIPO_FILE_BLOCK:
        return m_estado == EST_RECEIVING_FILE;

    case P::TIPO_FILE_END:
        return m_estado == EST_RECEIVING_FILE;

    case P::TIPO_FILE_ABORT:
        return m_estado == EST_RECEIVING_FILE;

    case P::TIPO_OP_START:
        // Basta NAO haver operacao em curso. A exigencia de arquivo validado
        // mudou de lugar -- ver tratarOpStart().
        //
        // POR QUE MUDOU: aqui so se enxerga o TIPO da mensagem, nunca o modo,
        // que vem no conteudo. Exigir EST_READY neste ponto valia para enviar e
        // quebrava a RECEPCAO: receber nao sobe arquivo nenhum, entao o estado e
        // IDLE e o comando era recusado com "invalido neste estado" -- uma
        // funcao inteira do produto inacessivel pelo aplicativo.
        return foraDeOperacao();

    case P::TIPO_OP_PAUSE:
        return m_estado == EST_SENDING || m_estado == EST_ONLINE;

    case P::TIPO_OP_RESUME:
        return m_estado == EST_PAUSED;

    case P::TIPO_OP_CANCEL:
        // Tambem nos dois estados de espera: o operador precisa poder desistir
        // enquanto a porta esta aberta aguardando a decisao dele.
        return m_estado == EST_SENDING || m_estado == EST_ONLINE
            || m_estado == EST_PAUSED || m_estado == EST_RECEIVING_FROM_CNC
            || m_estado == EST_AWAIT_FINISH || m_estado == EST_DRAIN_STALLED
            || m_estado == EST_RECEIVE_IDLE
            // Cancelar no erro e o gesto natural de quem quer limpar e
            // recomecar. Recusar isso era mandar o operador tirar da tomada.
            || m_estado == EST_ERROR;

    case P::TIPO_OP_FINISH:
        // Na RECEPCAO, finalizar significa "a maquina acabou, guarde o que
        // chegou". Mesmo gesto, sentido inverso do envio.
        if (m_estado == EST_RECEIVE_IDLE) return true;
        // SO quando o firmware ja disse que entregou tudo.
        //
        // Aceitar antes disso seria deixar o aplicativo fechar a porta com
        // programa ainda a caminho da maquina -- exatamente o que o estado de
        // espera existe para impedir. Quem quiser abandonar antes usa CANCEL,
        // que e explicito sobre o que esta fazendo.
        return m_estado == EST_AWAIT_FINISH;

    case P::TIPO_FILE_GET:
        // Ler um arquivo do cartao nao interfere na maquina, entao vale em
        // qualquer estado MENOS durante a recepcao da CNC -- ali o arquivo
        // ainda esta sendo escrito e mandaria um pedaco incompleto. O mesmo
        // vale na pausa da recepcao: a porta continua aberta e pode vir mais.
        return m_estado != EST_RECEIVING_FROM_CNC && m_estado != EST_RECEIVE_IDLE;

    case P::TIPO_FILE_LIST:
        // Listar nao toca na maquina nem no arquivo: vale sempre. Inclusive com
        // envio em curso, que e quando o operador quer justamente conferir o
        // espaco antes de mandar o proximo programa.
        return true;

    case P::TIPO_FILE_DELETE:
        // Apagar durante uma operacao cortaria o programa que esta sendo lido do
        // cartao em pedacos. A verificacao tambem existe dentro do tratador --
        // aqui para recusar cedo, la porque o estado pode mudar no meio.
        return foraDeOperacao();

    case P::TIPO_OP_CONTINUE:
        // Enquanto houver uma pergunta na tela esperando resposta: a drenagem
        // travada no envio, ou a maquina calada no meio da recepcao.
        return m_estado == EST_DRAIN_STALLED || m_estado == EST_RECEIVE_IDLE;

    default:
        return false;
    }
}

void DncProtocolCore::definirEstado(uint8_t novo)
{
    if (m_estado == novo) return;
    m_estado = novo;
    notificarEstado(novo);
}

// ---------------------------------------------------------------------------
// Despacho
// ---------------------------------------------------------------------------

void DncProtocolCore::tratarQuadro(const P::Quadro& q)
{
    if (q.versao != P::kVersaoAtual) {
        responderNack(q.opId, P::NACK_VERSAO);
        return;
    }

    // COMANDO REPETIDO NÃO EXECUTA DUAS VEZES.
    //
    // Se o comando chegou e só a resposta se perdeu, o app reenvia com o MESMO
    // opId. Aqui reconhecemos e respondemos de novo, sem executar. Sem isto,
    // um ACK perdido num "iniciar envio" mandaria o programa em dobro para a
    // máquina.
    if (q.opId != 0 && !m_dedup.registrarSeNovo(q.opId)) {
        responderAck(q.opId);
        return;
    }

    // Controle exclusivo: só um aparelho comanda. Consulta e sessão passam.
    //
    // O LIMITE SUPERIOR TEM QUE ACOMPANHAR OS COMANDOS NOVOS. Quando OP_FINISH
    // (0x34) e OP_CONTINUE (0x35) entraram, a faixa ainda terminava em
    // OP_CANCEL (0x33) -- e os dois ficavam de fora da exigência de controle,
    // ou seja, qualquer aparelho conectado poderia encerrar a sessão de outro.
    const bool exigeControle =
        (q.tipo >= P::TIPO_CONFIG_SET && q.tipo <= P::TIPO_OP_CONTINUE);
    if (exigeControle && !m_temControlador) {
        responderNack(q.opId, P::NACK_SEM_CONTROLE);
        return;
    }

    if (!comandoCabeNoEstado(q.tipo)) {
        ++m_recusasPorEstado;
        notificarLog(std::string("Comando ") + P::nomeDoTipo(q.tipo)
                     + " recusado no estado " + nomeDoEstado(m_estado));
        responderNack(q.opId, P::NACK_ESTADO_INVALIDO);
        return;
    }

    switch (q.tipo) {
    case P::TIPO_HELLO:      tratarHello(q);     break;
    case P::TIPO_CLAIM:      tratarClaim(q);     break;
    case P::TIPO_RELEASE:
        m_temControlador = false;
        m_idControlador.clear();
        responderAck(q.opId);
        break;
    case P::TIPO_PING: {
        P::Quadro r; r.tipo = P::TIPO_PONG; r.opId = q.opId;
        enviarQuadro(r);
        break;
    }
    case P::TIPO_STATE_QUERY:
        notificarEstado(m_estado);
        responderAck(q.opId);
        break;

    case P::TIPO_CONFIG_SET: tratarConfigSet(q); break;
    case P::TIPO_WIFI_SCAN:  tratarWifiScan(q);  break;
    case P::TIPO_WIFI_SET:   tratarWifiSet(q);   break;
    case P::TIPO_FILE_LIST:   tratarFileList(q);   break;
    case P::TIPO_FILE_DELETE: tratarFileDelete(q); break;
    case P::TIPO_FILE_BEGIN: tratarFileBegin(q); break;
    case P::TIPO_FILE_BLOCK: tratarFileBlock(q); break;
    case P::TIPO_FILE_END:   tratarFileEnd(q);   break;

    case P::TIPO_FILE_ABORT:
        m_arm.descartarTemporario();
        m_recebido = 0;
        definirEstado(EST_IDLE);
        responderAck(q.opId);
        break;

    case P::TIPO_OP_START:   tratarOpStart(q);   break;
    case P::TIPO_FILE_GET:   tratarFileGet(q);   break;

    case P::TIPO_OP_PAUSE:
        m_acoes.pausarOperacao();
        definirEstado(EST_PAUSED);
        responderAck(q.opId);
        break;

    case P::TIPO_OP_RESUME:
        m_acoes.retomarOperacao();
        definirEstado(EST_SENDING);
        responderAck(q.opId);
        break;

    case P::TIPO_OP_CANCEL:
        m_acoes.cancelarOperacao();
        definirEstado(EST_CANCELLED);
        responderAck(q.opId);
        break;

    case P::TIPO_OP_FINISH:
        m_acoes.finalizarOperacao();
        definirEstado(EST_COMPLETED);
        responderAck(q.opId);
        break;

    case P::TIPO_OP_CONTINUE:
        // NAO muda o estado aqui de proposito.
        //
        // Quem decide se a drenagem voltou a andar e o motor, olhando a fila da
        // UART. Marcar EST_SENDING neste ponto seria o protocolo afirmando um
        // progresso que ninguem observou -- e a tela mostraria "enviando" com a
        // maquina possivelmente ainda parada.
        m_acoes.continuarAguardando();
        responderAck(q.opId);
        break;

    default:
        responderNack(q.opId, P::NACK_NAO_SUPORTADO);
        break;
    }
}

void DncProtocolCore::tratarHello(const P::Quadro& q)
{
    std::vector<uint8_t> p;
    P::escreverU8(p, P::kVersaoAtual);
    P::escreverTexto(p, m_firmware);
    P::escreverTexto(p, m_idAparelho);
    P::escreverU8(p, m_linhas);
    P::escreverU32(p, P::kMaxPayload);
    P::escreverU64(p, m_arm.espacoLivre());

    // NO FIM DE PROPOSITO: aplicativo antigo le os campos que conhece e ignora
    // o resto, entao acrescentar aqui nao quebra quem ja esta instalado.
    //
    // 1 = caixa sem rede cadastrada (ou so com o ponto de acesso de socorro).
    // E por este byte que o aplicativo sabe que esta diante de uma instalacao
    // nova e deve oferecer a tela de escolher a rede do cliente.
    P::escreverU8(p, m_acoes.semRedeCadastrada() ? 1 : 0);

    P::Quadro r;
    r.tipo = P::TIPO_IDENTIFY;
    r.opId = q.opId;
    r.payload = p;
    enviarQuadro(r);
}

// --- Mandar um arquivo da caixa para o aplicativo ----------------------------

void DncProtocolCore::tratarFileGet(const P::Quadro& q)
{
    size_t pos = 0;
    std::string nome;
    if (!P::lerTexto(q.payload, pos, nome) || nome.empty()) {
        responderNack(q.opId, P::NACK_PARAMETRO);
        return;
    }

    // UM DOWNLOAD POR VEZ. Comecar outro no meio embaralharia os blocos dos
    // dois no mesmo canal, e o aplicativo montaria um arquivo com pedacos de
    // ambos -- corrompido sem nenhum erro aparente.
    if (m_enviandoArquivo) {
        responderNack(q.opId, P::NACK_ESTADO_INVALIDO);
        return;
    }

    uint64_t tamanho = 0;
    if (!m_arm.abrirParaLeitura(nome, tamanho)) {
        notificarLog("Pedido de arquivo inexistente: " + nome);
        responderNack(q.opId, P::NACK_ARQUIVO_INVALIDO);
        return;
    }

    // O tamanho vai ANTES dos dados para o aplicativo poder mostrar progresso
    // de verdade, e nao uma barra andando sem fim conhecido.
    std::vector<uint8_t> p;
    P::escreverTexto(p, nome);
    P::escreverU64(p, tamanho);

    P::Quadro r;
    r.tipo = P::TIPO_FILE_INFO;
    r.opId = q.opId;
    r.payload = p;
    enviarQuadro(r);

    m_enviandoArquivo = true;
    m_restanteDoEnvio = tamanho;
    m_seqDoEnvio = 0;
}

bool DncProtocolCore::continuarEnvioDeArquivo()
{
    if (!m_enviandoArquivo) return false;

    // Conexao caiu no meio: nao adianta continuar lendo o cartao para jogar
    // fora. Fecha e esquece.
    if (!m_trans.conectado()) {
        encerrarEnvioDeArquivo(false);
        return false;
    }

    // Um bloco por chamada. Ver a nota no cabecalho sobre nao despejar tudo de
    // uma vez.
    std::vector<uint8_t> bloco(1024);
    const size_t lidos = m_arm.lerPedaco(bloco.data(), bloco.size());

    if (lidos > 0) {
        bloco.resize(lidos);
        P::Quadro r;
        r.tipo = P::TIPO_FILE_DATA;
        r.seq = m_seqDoEnvio++;
        r.payload = bloco;
        enviarQuadro(r);

        if (m_restanteDoEnvio >= lidos) m_restanteDoEnvio -= lidos;
        else m_restanteDoEnvio = 0;
        return true;
    }

    // Acabou: manda o hash do que foi lido. O aplicativo so aceita o arquivo
    // depois de conferir -- mesma regra da subida, no sentido inverso.
    uint8_t hash[32] = {0};
    const bool temHash = m_arm.hashDaLeitura(hash);

    std::vector<uint8_t> p;
    P::escreverU8(p, temHash ? 1 : 0);
    P::escreverBytes(p, hash, 32);

    P::Quadro r;
    r.tipo = P::TIPO_FILE_DATA_END;
    r.payload = p;
    enviarQuadro(r);

    encerrarEnvioDeArquivo(true);
    return false;
}

void DncProtocolCore::encerrarEnvioDeArquivo(bool /*comSucesso*/)
{
    if (!m_enviandoArquivo) return;
    m_arm.fecharLeitura();
    m_enviandoArquivo = false;
    m_restanteDoEnvio = 0;
    m_seqDoEnvio = 0;
}

void DncProtocolCore::tratarClaim(const P::Quadro& q)
{
    size_t pos = 0;
    std::string id;
    P::lerTexto(q.payload, pos, id);

    // DOIS TABLETS NÃO COMANDAM A MESMA MÁQUINA.
    //
    // O segundo é recusado com motivo claro, não colocado numa fila. Numa
    // oficina, dois operadores mandando programa para o mesmo comando é
    // colisão garantida.
    if (m_temControlador && id != m_idControlador) {
        responderNack(q.opId, P::NACK_SEM_CONTROLE);
        return;
    }

    m_temControlador = true;
    m_idControlador = id;
    responderAck(q.opId);
    notificarEstado(m_estado);
}

void DncProtocolCore::tratarConfigSet(const P::Quadro& q)
{
    size_t pos = 0;
    ConfigSerial c;
    uint8_t dtr = 0, rts = 0;

    if (!P::lerU32(q.payload, pos, c.baud)
        || !P::lerU8(q.payload, pos, c.dataBits)
        || !P::lerU8(q.payload, pos, c.paridade)
        || !P::lerU8(q.payload, pos, c.stopBits)
        || !P::lerU8(q.payload, pos, c.fluxo)
        || !P::lerU8(q.payload, pos, dtr)
        || !P::lerU8(q.payload, pos, rts)
        || !P::lerU32(q.payload, pos, c.taxaManualBps)
        || !P::lerU32(q.payload, pos, c.chunkInicial)
        || !P::lerU32(q.payload, pos, c.chunkMin)
        || !P::lerU32(q.payload, pos, c.chunkMax)
        || !P::lerU32(q.payload, pos, c.holdSemConsumoMs)
        || !P::lerU32(q.payload, pos, c.debounceMs)) {
        responderNack(q.opId, P::NACK_PARAMETRO);
        return;
    }

    c.dtr = dtr != 0;
    c.rts = rts != 0;

    // Coerência dos blocos. O app já limita, mas quadro corrompido que passe
    // no CRC produziria um estado impossível que o motor resolveria em
    // silêncio — e comportamento silencioso é o que não se depura em campo.
    if (c.chunkMin > c.chunkMax) c.chunkMin = c.chunkMax;
    if (c.chunkInicial < c.chunkMin) c.chunkInicial = c.chunkMin;
    if (c.chunkInicial > c.chunkMax) c.chunkInicial = c.chunkMax;

    // So assume a configuracao se ela foi de fato aceita la embaixo.
    if (!m_acoes.aplicarConfigSerial(c)) {
        responderNack(q.opId, P::NACK_PARAMETRO);
        return;
    }
    m_config = c;
    responderAck(q.opId);
}

// ---------------------------------------------------------------------------
// Recepção de arquivo
// ---------------------------------------------------------------------------

// --- Gerenciar o cartao -----------------------------------------------------

void DncProtocolCore::tratarFileList(const P::Quadro& q)
{
    size_t pos = 0;
    std::string pasta;
    if (!P::lerTexto(q.payload, pos, pasta)) {
        responderNack(q.opId, P::NACK_PARAMETRO);
        return;
    }

    const auto arquivos = m_arm.listar(pasta);

    std::vector<uint8_t> p;
    // Espaco junto da lista, de proposito: a tela precisa dos dois ao mesmo
    // tempo e uma viagem so evita mostrar lista nova com numero velho.
    P::escreverU64(p, m_arm.espacoLivre());
    P::escreverU64(p, m_arm.espacoTotal());

    // Teto de 200: o quadro tem limite, e uma lista maior que isso nao se le
    // numa tela de chao de fabrica. Quem lista ja devolve o maior primeiro.
    const uint16_t quantos = (uint16_t)(arquivos.size() > 200 ? 200 : arquivos.size());
    P::escreverU16(p, quantos);
    for (uint16_t i = 0; i < quantos; ++i) {
        P::escreverTexto(p, arquivos[i].nome);
        P::escreverU32(p, arquivos[i].bytes);
    }

    P::Quadro r;
    r.tipo = P::TIPO_FILE_LIST_RESP;
    r.opId = q.opId;
    r.payload = p;
    enviarQuadro(r);
}

void DncProtocolCore::tratarFileDelete(const P::Quadro& q)
{
    size_t pos = 0;
    std::string pasta, nome;
    if (!P::lerTexto(q.payload, pos, pasta) || !P::lerTexto(q.payload, pos, nome)) {
        responderNack(q.opId, P::NACK_PARAMETRO);
        return;
    }
    if (nome.empty()) {
        responderNack(q.opId, P::NACK_PARAMETRO);
        return;
    }
    // APAGAR O QUE ESTA SENDO ENVIADO AGORA CORTARIA O PROGRAMA NA MAQUINA.
    //
    // A leitura e feita em pedacos ao longo de toda a transmissao: sumir com o
    // arquivo no meio nao para o envio, deixa a maquina com meio programa.
    if (!foraDeOperacao()) {
        notificarLog("Pedido para apagar arquivo durante operacao: recusado.");
        responderNack(q.opId, P::NACK_ESTADO_INVALIDO);
        return;
    }
    if (!m_arm.apagar(pasta, nome)) {
        responderNack(q.opId, P::NACK_ARMAZENAMENTO);
        return;
    }
    responderAck(q.opId);
}

// --- Rede sem fio -----------------------------------------------------------

void DncProtocolCore::tratarWifiScan(const P::Quadro& q)
{
    const auto redes = m_acoes.redesVisiveis();

    std::vector<uint8_t> p;
    // Teto de 20: o quadro tem limite e uma lista gigante nao ajuda ninguem a
    // escolher. Quem varre ja devolve ordenado por sinal.
    const uint8_t quantas = (uint8_t)(redes.size() > 20 ? 20 : redes.size());
    P::escreverU8(p, quantas);
    for (uint8_t i = 0; i < quantas; ++i) {
        P::escreverTexto(p, redes[i].ssid);
        P::escreverU8(p, (uint8_t)(int8_t)redes[i].sinal);
        P::escreverU8(p, redes[i].protegida ? 1 : 0);
    }

    P::Quadro r;
    r.tipo = P::TIPO_WIFI_LIST;
    r.opId = q.opId;
    r.payload = p;
    enviarQuadro(r);
}

void DncProtocolCore::tratarWifiSet(const P::Quadro& q)
{
    size_t pos = 0;
    std::string ssid, senha;
    if (!P::lerTexto(q.payload, pos, ssid) || !P::lerTexto(q.payload, pos, senha)) {
        responderNack(q.opId, P::NACK_PARAMETRO);
        return;
    }
    // SSID vazio apagaria a rede e tiraria a caixa do ar -- a mesma guarda que
    // o portal web ja tem. Rede aberta (senha vazia) continua valendo.
    if (ssid.empty() || ssid.size() > 32 || senha.size() > 63) {
        responderNack(q.opId, P::NACK_PARAMETRO);
        return;
    }
    if (!m_acoes.gravarRede(ssid, senha)) {
        responderNack(q.opId, P::NACK_ARMAZENAMENTO);
        return;
    }
    // ACK ANTES de trocar de rede: assim que a caixa tentar entrar na rede
    // nova, esta conexao morre junto com o ponto de acesso de socorro. Se a
    // resposta fosse depois, o aplicativo nunca a receberia e mostraria falha
    // num cadastro que deu certo.
    responderAck(q.opId);
}

void DncProtocolCore::tratarFileBegin(const P::Quadro& q)
{
    size_t pos = 0;
    std::string nome;
    uint64_t tamanho = 0;

    if (!P::lerTexto(q.payload, pos, nome) || !P::lerU64(q.payload, pos, tamanho)) {
        responderNack(q.opId, P::NACK_PARAMETRO);
        return;
    }
    if (pos + 32 > q.payload.size()) {
        responderNack(q.opId, P::NACK_PARAMETRO);
        return;
    }
    std::memcpy(m_hashEsperado, q.payload.data() + pos, 32);

    if (tamanho == 0) {
        responderNack(q.opId, P::NACK_PARAMETRO);
        return;
    }
    if (tamanho > m_arm.espacoLivre()) {
        notificarLog("Espaco insuficiente no cartao para receber o programa.");
        responderNack(q.opId, P::NACK_ARMAZENAMENTO);
        return;
    }
    if (!m_arm.abrirTemporario()) {
        responderNack(q.opId, P::NACK_ARMAZENAMENTO);
        return;
    }

    m_nomeRecebendo   = nome;
    m_tamanhoEsperado = tamanho;
    m_recebido        = 0;
    m_proximaSeq      = 0;

    definirEstado(EST_RECEIVING_FILE);
    responderAck(q.opId);
}

void DncProtocolCore::tratarFileBlock(const P::Quadro& q)
{
    // ORDEM IMPORTA.
    //
    // Bloco fora de sequência significa que algo se perdeu. Gravar assim
    // montaria um arquivo com buraco — que o hash pegaria no fim, depois de
    // subir tudo à toa. Recusar aqui faz o app corrigir na hora.
    if (q.seq != m_proximaSeq) {
        notificarLog("Bloco fora de ordem: esperado outro numero.");
        responderNack(q.opId, P::NACK_BLOCO_FORA_ORDEM);
        return;
    }

    if (m_recebido + q.payload.size() > m_tamanhoEsperado) {
        notificarLog("Bloco excede o tamanho anunciado.");
        responderNack(q.opId, P::NACK_ARQUIVO_INVALIDO);
        return;
    }

    if (!m_arm.escreverTemporario(q.payload.data(), q.payload.size())) {
        m_arm.descartarTemporario();
        definirEstado(EST_ERROR);
        responderNack(q.opId, P::NACK_ARMAZENAMENTO);
        return;
    }

    m_recebido += q.payload.size();
    ++m_proximaSeq;
    responderAck(q.opId);
}

void DncProtocolCore::tratarFileEnd(const P::Quadro& q)
{
    definirEstado(EST_VALIDATING_FILE);

    if (!m_arm.fecharTemporario()) {
        m_arm.descartarTemporario();
        definirEstado(EST_ERROR);
        responderNack(q.opId, P::NACK_ARMAZENAMENTO);
        return;
    }

    // TAMANHO E HASH, NESTA ORDEM, ANTES DE PROMOVER.
    //
    // O definitivo só passa a existir depois disto. Queda no meio do upload
    // deixa um temporário órfão, nunca um programa aparentemente bom.
    if (!m_arm.validarTemporario(m_hashEsperado, m_tamanhoEsperado)) {
        m_arm.descartarTemporario();
        notificarLog("Arquivo recebido nao confere: tamanho ou hash.");
        definirEstado(EST_IDLE);
        responderNack(q.opId, P::NACK_ARQUIVO_INVALIDO);
        return;
    }

    if (!m_arm.promoverTemporario(m_nomeRecebendo)) {
        m_arm.descartarTemporario();
        definirEstado(EST_ERROR);
        responderNack(q.opId, P::NACK_ARMAZENAMENTO);
        return;
    }

    m_arquivoPronto = m_nomeRecebendo;
    definirEstado(EST_READY);
    responderAck(q.opId);
    notificarLog("Programa recebido e validado: " + m_nomeRecebendo);
}

void DncProtocolCore::tratarOpStart(const P::Quadro& q)
{
    size_t pos = 0;
    uint8_t modo = 0;
    std::string nome;

    if (!P::lerU8(q.payload, pos, modo) || !P::lerTexto(q.payload, pos, nome)) {
        responderNack(q.opId, P::NACK_PARAMETRO);
        return;
    }

    // AS TRAVAS ABAIXO SO VALEM PARA ENVIO. Modo 2 e recepcao da CNC.
    //
    // Na recepcao nao existe arquivo de origem: o `nome` e o DESTINO onde
    // gravar o que a maquina mandar. Aplicar a conferencia de "arquivo
    // validado" aqui recusava toda recepcao -- primeiro pelo estado, depois
    // pelo nome.
    const bool recepcao = (modo == 2);

    if (!recepcao) {
        // Enviar sem arquivo validado seria mandar para a maquina um programa
        // que nunca passou por conferencia de tamanho e hash.
        if (m_estado != EST_READY) {
            notificarLog("Pedido de envio sem arquivo validado.");
            responderNack(q.opId, P::NACK_ESTADO_INVALIDO);
            return;
        }
        if (nome != m_arquivoPronto) {
            notificarLog("Pedido de envio de arquivo diferente do validado.");
            responderNack(q.opId, P::NACK_ARQUIVO_INVALIDO);
            return;
        }
    }

    if (!m_acoes.iniciarOperacao(modo, nome)) {
        definirEstado(EST_ERROR);
        responderNack(q.opId, P::NACK_ARMAZENAMENTO);
        return;
    }

    definirEstado(modo == 1 ? EST_ONLINE
                : modo == 2 ? EST_RECEIVING_FROM_CNC
                            : EST_SENDING);
    responderAck(q.opId);
}

// ---------------------------------------------------------------------------
// Saída
// ---------------------------------------------------------------------------

void DncProtocolCore::enviarQuadro(const P::Quadro& q)
{
    if (!m_trans.conectado()) return;
    const auto bytes = P::codificar(q);
    if (bytes.empty()) return;
    m_trans.enviar(bytes.data(), bytes.size());
}

void DncProtocolCore::responderAck(uint32_t opId)
{
    const auto b = P::codificarAck(opId);
    if (m_trans.conectado()) m_trans.enviar(b.data(), b.size());
}

void DncProtocolCore::responderNack(uint32_t opId, P::MotivoNack motivo)
{
    const auto b = P::codificarNack(opId, motivo);
    if (m_trans.conectado()) m_trans.enviar(b.data(), b.size());
}

void DncProtocolCore::notificarEstado(uint8_t estado)
{
    P::Quadro q;
    q.tipo = P::TIPO_EVENT_STATE;
    P::escreverU8(q.payload, estado);
    enviarQuadro(q);
}

void DncProtocolCore::notificarProgresso(uint64_t enviados, uint64_t total, uint32_t bps)
{
    P::Quadro q;
    q.tipo = P::TIPO_EVENT_PROGRESS;
    P::escreverU64(q.payload, enviados);
    P::escreverU64(q.payload, total);
    P::escreverU32(q.payload, bps);
    enviarQuadro(q);
}

void DncProtocolCore::notificarSinais(uint8_t bitmask)
{
    P::Quadro q;
    q.tipo = P::TIPO_EVENT_SIGNALS;
    P::escreverU8(q.payload, bitmask);
    enviarQuadro(q);
}

void DncProtocolCore::notificarLog(const std::string& texto)
{
    P::Quadro q;
    q.tipo = P::TIPO_EVENT_LOG;
    P::escreverTexto(q.payload, texto);
    enviarQuadro(q);
}

void DncProtocolCore::notificarErro(uint8_t motivo, const std::string& texto)
{
    P::Quadro q;
    q.tipo = P::TIPO_EVENT_ERROR;
    P::escreverU8(q.payload, motivo);
    P::escreverTexto(q.payload, texto);
    enviarQuadro(q);
}

} // namespace esp
} // namespace dnc
} // namespace smi
