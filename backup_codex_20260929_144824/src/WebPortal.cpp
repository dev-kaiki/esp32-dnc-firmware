#include <esp_task_wdt.h>
#include "WebPortal.h"
#include <WiFi.h>
#include "Logger.h"
#include "../Config.h"
#include "CircularBuffer.h"
#include "OtaUpdater.h"
#include <esp_ota_ops.h>

// Definida no sketch. Ver a nota la sobre por que a particao e a prova de que
// uma atualizacao pelo ar pegou.
extern String particaoEmUso();

// Respostas do operador a sessao de envio. Definidas no sketch porque e la que
// mora o motor e a sua tranca.
extern void finalizarSessaoPeloOperador();
extern void continuarAguardandoDrenagem();

extern smi::dnc::esp::OtaUpdater gOta;
extern QueueHandle_t gDncCommandQueue;
extern CircularBuffer txRing;
extern SystemStatus gStatus;
extern SemaphoreHandle_t gStatusMutex;
extern DncSerialConfig gSerialConfig;
extern NetworkConfig gNetworkConfig;
extern AccessConfig gAccessConfig;
extern Settings settings;
extern StorageManager storage;
extern String gRecentLogs[MAX_RECENT_LOGS];
extern uint16_t gRecentLogHead;
extern SemaphoreHandle_t gLogMutex;

WebPortal webPortal;

static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html><html lang="pt-BR"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SMI DNC</title><style>
:root{--bg:#121212;--surf:#1a1a1a;--surf2:#1f1f1f;--field:#242424;--b:#333;--txt:#fff;--body:#e0e0e0;--dim:#9e9e9e;--faint:#7a7a7a;--ac:#2e7d32;--ac2:#66bb6a;--warn:#ef6c00;--warn2:#ffb74d;--dan:#c62828}
*{box-sizing:border-box}
html{-webkit-text-size-adjust:100%}
body{margin:0;background:var(--bg);color:var(--body);font-family:system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif;font-size:15px;line-height:1.45}
/* ---- cabecalho fixo, com o estado sempre a vista ---- */
.top{position:sticky;top:0;z-index:9;background:var(--surf);border-bottom:1px solid var(--b);box-shadow:0 2px 10px #0008}
.top .bar{display:flex;align-items:center;gap:14px;padding:10px 16px;flex-wrap:wrap}
.brand{display:flex;align-items:center;gap:9px;font-weight:700;font-size:18px;color:var(--txt);letter-spacing:.3px}
.mark{width:30px;height:30px;border-radius:7px;background:linear-gradient(140deg,var(--ac2),var(--ac));display:grid;place-items:center;color:#08130a;font-weight:800;font-size:12px;letter-spacing:-.5px}
.brand small{display:block;font-weight:400;font-size:11px;color:var(--dim);letter-spacing:.6px;text-transform:uppercase}
.pills{display:flex;gap:8px;flex-wrap:wrap;margin-left:auto;align-items:center}
.pill{background:var(--surf2);border:1px solid var(--b);border-radius:999px;padding:5px 12px;font-size:12.5px;display:flex;gap:7px;align-items:center;white-space:nowrap}
.pill b{color:var(--txt);font-weight:600}
.pill .k{color:var(--faint);text-transform:uppercase;font-size:10.5px;letter-spacing:.7px}
#stState{font-weight:700;color:var(--ac2)}
/* ---- estrutura ---- */
.wrap{display:grid;grid-template-columns:212px 1fr;min-height:calc(100vh - 52px)}
nav{background:var(--surf);border-right:1px solid var(--b);padding:12px;display:flex;flex-direction:column;gap:5px}
nav button{width:100%;padding:12px 14px;border:1px solid transparent;border-radius:9px;background:transparent;color:var(--body);text-align:left;cursor:pointer;font-size:14.5px;font-family:inherit;transition:background .12s,color .12s}
nav button:hover{background:var(--field);color:var(--txt)}
nav button.active{background:var(--ac);color:#fff;font-weight:600}
main{padding:20px;max-width:1100px}
section>h2{margin:0 0 4px;font-size:20px;color:var(--txt);font-weight:600}
section>h2+p.muted{margin-top:0}
h2+.grid,h2+.card,h2+.row{margin-top:14px}
/* ---- cartoes ---- */
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(215px,1fr));gap:12px}
.card{background:var(--surf);border:1px solid var(--b);border-radius:11px;padding:14px 16px}
.card h3{margin:0 0 9px;font-size:11.5px;font-weight:600;color:var(--faint);text-transform:uppercase;letter-spacing:.9px}
.card label{display:block;font-size:11.5px;font-weight:600;color:var(--faint);text-transform:uppercase;letter-spacing:.9px;margin-bottom:7px}
.value{font-size:21px;font-weight:700;color:var(--txt);line-height:1.25}
.muted{color:var(--dim);font-size:13.5px}
.row{display:flex;gap:9px;flex-wrap:wrap;align-items:center;margin:11px 0}
/* ---- controles ---- */
button,.btn{background:var(--ac);color:#fff;border:0;border-radius:9px;padding:11px 16px;font-weight:600;font-size:14.5px;font-family:inherit;cursor:pointer;text-decoration:none;display:inline-block;min-height:42px}
button:hover,.btn:hover{filter:brightness(1.14)}
button:active{transform:translateY(1px)}
button.warn{background:var(--warn)}
button.danger{background:var(--dan)}
button.gray{background:var(--field);color:var(--body);border:1px solid var(--b)}
input,select{background:var(--field);color:var(--txt);border:1px solid var(--b);border-radius:9px;padding:11px 12px;font-size:14.5px;font-family:inherit;min-height:42px;width:100%}
input:focus,select:focus{outline:0;border-color:var(--ac2)}
input[type=file]{padding:8px}
.row input{width:auto;flex:1 1 170px}
/* ---- tabela de arquivos ---- */
table{width:100%;border-collapse:collapse;background:var(--surf);border:1px solid var(--b);border-radius:11px;overflow:hidden;font-size:14px}
th{background:var(--surf2);color:var(--faint);font-size:11.5px;text-transform:uppercase;letter-spacing:.8px}
td,th{border-bottom:1px solid var(--b);padding:11px 12px;text-align:left}
tr:last-child td{border-bottom:0}
tbody tr:hover{background:var(--surf2)}
td button{padding:7px 11px;font-size:13px;min-height:34px}
/* ---- progresso ---- */
progress{width:100%;height:14px;border:0;border-radius:999px;overflow:hidden;background:var(--field);-webkit-appearance:none;appearance:none}
progress::-webkit-progress-bar{background:var(--field)}
progress::-webkit-progress-value{background:linear-gradient(90deg,var(--ac),var(--ac2))}
progress::-moz-progress-bar{background:var(--ac2)}
/* ---- estados ---- */
.ok{color:var(--ac2);font-weight:600}
.repouso{color:var(--dim);font-weight:600}
.bad{color:#ff7b72;font-weight:600}
.log{height:min(58vh,430px);overflow:auto;background:#0d0d0d;border:1px solid var(--b);border-radius:10px;padding:12px;font-family:ui-monospace,Consolas,"Courier New",monospace;font-size:12.5px;line-height:1.6;white-space:pre-wrap;color:#cfd6cf}
.hidden{display:none}
.aviso{background:#2a1410;border:1px solid var(--dan);border-left:4px solid var(--dan);border-radius:9px;padding:12px 15px;margin:12px 0}
.aviso b{color:#ff9d95}
.nota{background:var(--surf);border-left:3px solid var(--b);border-radius:0 9px 9px 0;padding:11px 15px;margin:12px 0;color:var(--dim);font-size:13.5px}
.nota b{color:var(--body)}
/* ---- celular ---- */
@media(max-width:820px){
 .wrap{grid-template-columns:1fr}
 nav{flex-direction:row;overflow-x:auto;border-right:0;border-bottom:1px solid var(--b);padding:9px;gap:7px}
 nav button{width:auto;flex:0 0 auto;padding:10px 15px;white-space:nowrap}
 main{padding:14px}
 .pills{width:100%;margin-left:0}
 .top .bar{padding:9px 13px;gap:9px}
}
</style></head><body>
<header class="top"><div class="bar">
<div class="brand"><div class="mark">SMI</div><div>SMI DNC<small>Transferencia CNC</small></div></div>
<div class="pills">
<span class="pill"><span class="k">Estado</span><span id="stState">-</span></span>
<span class="pill"><span class="k">RS232</span><span id="stSerial">-</span></span>
<span class="pill"><span class="k">Rede</span><span id="stNet">-</span><b id="stIp">-</b></span>
<button class="gray" onclick="logout()">Sair</button>
</div></div></header>
<div class="wrap"><nav>
<button onclick="tab('dash')" class="active">Painel</button><button onclick="tab('files')">Arquivos</button><button onclick="tab('transfer')">Transferencia</button><button onclick="tab('serial')">Serial RS232</button><button onclick="tab('network')">Rede</button><button onclick="tab('logs')">Registros</button></nav><main>

<section id="dash">
<h2>Painel</h2>
<p class="muted">Situacao deste equipamento agora.</p>
<div class="grid">
<div class="card"><h3>Programa</h3><div class="value" id="stFile">-</div></div>
<div class="card"><h3>Controle de fluxo</h3><div id="stFlow">-</div></div>
<div class="card"><h3>Cartao microSD</h3><div id="stSd">-</div><div class="muted" id="stSpace">-</div></div>
<div class="card"><h3>Firmware</h3><div class="value" id="stVer">-</div><div class="muted" id="stPart">-</div></div>
</div>
<div class="card" style="margin-top:12px"><h3>Progresso da transferencia</h3><progress id="prog" value="0" max="100"></progress><div class="muted" id="stProg" style="margin-top:8px">0%</div></div>
</section>

<section id="files" class="hidden"><h2>Arquivos</h2>
<p class="muted">Programas guardados no cartao do equipamento.</p>
<div class="row"><button class="gray" onclick="upDir()">Voltar</button><input id="cwd" value="/programs"><button onclick="loadFiles()">Abrir</button></div>
<div class="row"><input id="mkdirName" placeholder="Nome da nova pasta"><button class="gray" onclick="mkdir()">Criar pasta</button></div>
<div class="row"><input type="file" id="up"><button onclick="upload()">Enviar arquivo</button></div>
<table><thead><tr><th>Nome</th><th>Tipo</th><th>Tamanho</th><th>Acoes</th></tr></thead><tbody id="fileRows"></tbody></table></section>

<section id="transfer" class="hidden"><h2>Transferencia / DNC Online</h2>
<p class="muted">Escolha o programa e comande a transferencia com a maquina.</p>
<div class="card">
<label>Programa</label>
<div class="row"><input id="cmdPath" placeholder="/programs/arquivo.nc" style="flex:1 1 280px"><button onclick="cmd('connect')">Conectar</button><button class="gray" onclick="cmd('disconnect')">Desconectar</button></div>
<div class="row"><button onclick="cmd('send')">Enviar para CNC</button><button onclick="cmd('receive')">Receber da CNC</button><button onclick="cmd('drip')">Executar Online</button></div>
<div class="row"><button class="warn" onclick="cmd('pause')">Pausar</button><button class="gray" onclick="cmd('resume')">Retomar</button><button class="danger" onclick="cmd('cancel')">Cancelar</button></div>
<div class="row"><button onclick="cmd('finish')">Confirmar encerramento</button><button class="gray" onclick="cmd('continue')">Continuar aguardando</button></div>
</div>
<div class="nota"><b>O programa sair pela serial nao significa que a maquina terminou de usinar.</b> Ao terminar de transferir, o equipamento aguarda em <b>Transferido - aguardando confirmacao</b>: confirme o encerramento so depois que a maquina tiver concluido. Se aparecer <b>A maquina parou de consumir</b>, a porta continua aberta - use <b>Continuar aguardando</b>.</div>
<div class="nota">Para receber, o caminho de destino e opcional. Em branco, o programa e salvo em <b>/received</b>.</div>
</section>

<section id="serial" class="hidden"><h2>Serial RS232</h2>
<p class="muted">Precisa ser igual ao que esta configurado na maquina.</p>
<div class="grid">
<div class="card"><label>Baud rate</label><input id="baud" value="9600"></div>
<div class="card"><label>Bits de dados</label><select id="dataBits"><option>5</option><option>6</option><option selected>7</option><option>8</option></select></div>
<div class="card"><label>Paridade</label><select id="parity"><option value="0">Nenhuma</option><option value="1" selected>Par</option><option value="2">Impar</option></select></div>
<div class="card"><label>Bits de parada</label><select id="stopBits"><option selected>1</option><option>2</option></select></div>
<div class="card"><label>Tempo de espera (ms)</label><input id="timeout" value="3000"></div>
<div class="card"><label>Controle de fluxo</label><select id="flow"><option value="0">Nenhum</option><option value="1" selected>XON/XOFF</option><option value="2">RTS/CTS</option><option value="3">DTR/DSR</option></select></div>
</div>
<div class="row"><button onclick="saveSerial()">Salvar e aplicar</button></div></section>

<section id="network" class="hidden"><h2>Identificacao deste equipamento</h2>
<p class="muted">O nome aparece na lista do roteador e na atualizacao de firmware.</p>
<div class="grid">
<div class="card"><label>Nome do equipamento</label><input id="hostname" placeholder="ex.: smi-centur"></div>
<div class="card"><label>Como ficou</label><div id="hostnameReal" class="value">-</div></div>
</div>
<div class="nota"><b>De um nome diferente a cada caixa.</b> Com nomes iguais da para gravar firmware na maquina errada sem perceber. Só letras, numeros e traço: acento, espaço e ponto sao convertidos. Vale apos reiniciar.</div>

<h2>Rede Wi-Fi</h2>
<div class="grid">
<div class="card"><label>Nome da rede (SSID)</label><input id="ssid"></div>
<div class="card"><label>Senha do Wi-Fi</label><input id="wpass" type="password"></div>
<div class="card"><label>Endereco</label><select id="dhcp"><option value="1">Automatico (DHCP)</option><option value="0">Fixo</option></select></div>
<div class="card"><label>IP</label><input id="ip" placeholder="192.168.1.220"></div>
<div class="card"><label>Gateway</label><input id="gw" placeholder="192.168.1.1"></div>
<div class="card"><label>Mascara</label><input id="mask" placeholder="255.255.255.0"></div>
<div class="card"><label>DNS</label><input id="dns" placeholder="8.8.8.8"></div>
</div>

<h2>Atualizacao pelo Wi-Fi</h2>
<div class="grid">
<div class="card"><label>Senha de atualizacao</label><input id="otapass" type="password" placeholder="em branco = nao alterar"></div>
<div class="card"><label>Situacao</label><div id="otastat" class="muted">-</div></div>
</div>
<div class="row"><button onclick="saveNetwork()">Salvar rede</button></div>
<div class="nota">Reinicie o equipamento apos alterar a rede. Sem senha de atualizacao o recurso fica desligado, e a atualizacao e recusada enquanto houver transferencia em andamento.</div>

<h2>Senhas de acesso</h2>
<div id="avisoSenha" class="hidden"><div class="aviso"><b>Este equipamento ainda esta com senha de fabrica.</b> Ela e igual em todos os aparelhos: quem souber a de um entra em todos e, por esta mesma aba, chega na senha de atualizacao de firmware. Defina senhas proprias e anote-as na etiqueta do equipamento.</div></div>
<div class="grid">
<div class="card"><label>Administrador (admin)</label><input id="padm" type="password" placeholder="em branco = nao alterar"></div>
<div class="card"><label>Operador (operador)</label><input id="pope" type="password" placeholder="em branco = nao alterar"></div>
<div class="card"><label>Consulta (viewer)</label><input id="pview" type="password" placeholder="em branco = nao alterar"></div>
</div>
<div class="row"><button onclick="saveSenhas()">Salvar senhas</button></div>
<div class="nota">Minimo de 4 caracteres, nomes de usuario fixos. A sessao aberta continua valida apos a troca. <b>Anote a senha antes de salvar</b> - nao ha como recupera-la pelo portal.</div>
</section>

<section id="logs" class="hidden"><h2>Registros</h2>
<p class="muted">O que o equipamento fez, do mais antigo ao mais recente.</p>
<div class="row"><button onclick="loadLogs()">Atualizar</button><a class="btn gray" href="/download?path=/logs/dnc.log">Baixar arquivo completo</a></div>
<div id="logBox" class="log"></div></section>

</main></div>
<script>
let current='/programs';function qs(id){return document.getElementById(id)}function tab(id){document.querySelectorAll('main section').forEach(s=>s.classList.add('hidden'));qs(id).classList.remove('hidden');document.querySelectorAll('nav button').forEach(b=>b.classList.remove('active'));event.target.classList.add('active');if(id==='files')loadFiles();if(id==='serial')loadSerial();if(id==='network')loadNetwork();if(id==='logs')loadLogs()}
async function get(u){let r=await fetch(u);if(r.status===401)location='/login';return r.json()}async function post(u,d){let r=await fetch(u,{method:'POST',body:new URLSearchParams(d)});if(r.status===401)location='/login';return r.json()}
function fmt(n){n=Number(n||0);if(n>1073741824)return(n/1073741824).toFixed(2)+' GB';if(n>1048576)return(n/1048576).toFixed(2)+' MB';if(n>1024)return(n/1024).toFixed(1)+' KB';return n+' B'}
async function loadStatus(){let s=await get('/api/status');qs('stState').textContent=s.stateText;qs('stFile').textContent=s.currentFile||'-';qs('stSerial').innerHTML=s.serialConnected?'<span class="ok">Ativa</span>':'<span class="repouso">Em repouso</span>';qs('stFlow').textContent='Fluxo: '+s.flowText+' | CTS '+(s.ctsOk?'OK':'BLOQ')+' | DSR '+(s.dsrOk?'OK':'BLOQ')+(s.flowPaused?' | PAUSADO':'');qs('stSd').innerHTML=s.sdOk?'<span class="ok">OK</span>':'<span class="bad">Falha</span>';qs('stSpace').textContent='Livre: '+fmt(s.freeBytes)+' / Total: '+fmt(s.totalBytes);qs('stNet').innerHTML=s.wifiOk?'<span class="ok">Online</span>':(s.apMode?'AP Local':'Offline');qs('stIp').textContent=s.ip||'-';qs('stVer').textContent='v'+(s.versao||'?');qs('stPart').textContent='Partição '+(s.particao||'?')+' | '+(s.build||'');let pct=s.fileSize?Math.floor((s.bytesSent*100)/s.fileSize):0;qs('prog').value=pct;qs('stProg').textContent=pct+'% | Enviado '+fmt(s.bytesSent)+' / '+fmt(s.fileSize)+' | RX '+fmt(s.bytesReceived)+' | '+fmt(s.bps)+'/s'}
async function loadFiles(){current=qs('cwd').value||current;let j=await get('/api/files?path='+encodeURIComponent(current));current=j.path;qs('cwd').value=current;let rows='';j.items.forEach(f=>{rows+=`<tr><td>${f.dir?'📁':'📄'} ${f.name}</td><td>${f.dir?'Pasta':'Arquivo'}</td><td>${fmt(f.size)}</td><td><button onclick="openItem('${f.path}',${f.dir})">Abrir</button> <button onclick="pick('${f.path}')">Selecionar</button> <button class="gray" onclick="ren('${f.path}')">Renomear</button> <button class="danger" onclick="delp('${f.path}')">Excluir</button> <a class="btn" href="/download?path=${encodeURIComponent(f.path)}">Download</a></td></tr>`});qs('fileRows').innerHTML=rows}
function openItem(p,d){if(d){current=p;qs('cwd').value=p;loadFiles()}}function pick(p){qs('cmdPath').value=p;tabDirect('transfer')}function tabDirect(id){document.querySelectorAll('main section').forEach(s=>s.classList.add('hidden'));qs(id).classList.remove('hidden')}function upDir(){let p=current.split('/');p.pop();current=p.join('/')||'/';qs('cwd').value=current;loadFiles()}async function mkdir(){await post('/api/mkdir',{path:current+'/'+qs('mkdirName').value});loadFiles()}async function delp(p){if(confirm('Excluir '+p+'?')){await post('/api/delete',{path:p});loadFiles()}}async function ren(p){let n=prompt('Novo caminho completo:',p);if(n)await post('/api/rename',{from:p,to:n});loadFiles()}async function upload(){let f=qs('up').files[0];if(!f)return alert('Selecione arquivo');let fd=new FormData();fd.append('dir',current);fd.append('file',f);let r=await fetch('/api/upload',{method:'POST',body:fd});alert(await r.text());loadFiles()}
async function cmd(a){let p=qs('cmdPath').value;let j=await post('/api/cmd',{action:a,path:p});alert(j.ok?'Comando aceito':j.error)}
async function loadSerial(){let s=await get('/api/serial');qs('baud').value=s.baudRate;qs('dataBits').value=s.dataBits;qs('parity').value=s.parity;qs('stopBits').value=s.stopBits;qs('timeout').value=s.timeoutMs;qs('flow').value=s.flow}async function saveSerial(){let j=await post('/api/serial',{baudRate:qs('baud').value,dataBits:qs('dataBits').value,parity:qs('parity').value,stopBits:qs('stopBits').value,timeoutMs:qs('timeout').value,flow:qs('flow').value});alert(j.ok?'Serial aplicado':j.error)}
async function loadNetwork(){let n=await get('/api/network');qs('hostname').value=n.nome;qs('hostnameReal').textContent=n.nome;qs('ssid').value=n.ssid;qs('dhcp').value=n.dhcp?'1':'0';qs('ip').value=n.ip;qs('gw').value=n.gateway;qs('mask').value=n.subnet;qs('dns').value=n.dns;qs('otastat').textContent=n.otaAtivo?('Ativo como "'+n.otaNome+'"'):(n.otaSenhaDefinida?'Senha definida, aguardando reinício':'Desligado (sem senha)');qs('avisoSenha').className=n.senhaDeFabrica?'':'hidden'}
async function saveSenhas(){let j=await post('/api/senhas',{admin:qs('padm').value,operador:qs('pope').value,viewer:qs('pview').value});if(j.ok){qs('padm').value='';qs('pope').value='';qs('pview').value='';alert('Senhas salvas. Anote-as na etiqueta do equipamento.');loadNetwork()}else{alert(j.error)}}async function saveNetwork(){let j=await post('/api/network',{nome:qs('hostname').value,ssid:qs('ssid').value,password:qs('wpass').value,otapass:qs('otapass').value,dhcp:qs('dhcp').value,ip:qs('ip').value,gateway:qs('gw').value,subnet:qs('mask').value,dns:qs('dns').value});alert(j.ok?'Rede salva. Reinicie.':j.error)}
async function loadLogs(){let l=await get('/api/logs');qs('logBox').textContent=l.lines.join('\n');qs('logBox').scrollTop=qs('logBox').scrollHeight}function logout(){location='/logout'}setInterval(loadStatus,1000);loadStatus();
</script></body></html>
)HTML";

static const char LOGIN_HTML[] PROGMEM = R"HTML(<!doctype html><html lang="pt-BR"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SMI DNC</title><style>
:root{--bg:#121212;--surf:#1a1a1a;--field:#242424;--b:#333;--txt:#fff;--body:#e0e0e0;--dim:#9e9e9e;--faint:#7a7a7a;--ac:#2e7d32;--ac2:#66bb6a;--dan:#c62828}
*{box-sizing:border-box}
body{margin:0;min-height:100vh;background:radial-gradient(1100px 520px at 50% -8%,#1b2a1d 0%,var(--bg) 62%);color:var(--body);font-family:system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif;font-size:15px;display:grid;place-items:center;padding:20px}
.box{width:min(400px,100%);background:var(--surf);border:1px solid var(--b);border-radius:15px;padding:30px 26px;box-shadow:0 18px 50px #000a}
.brand{display:flex;align-items:center;gap:11px;margin-bottom:6px}
.mark{width:46px;height:46px;border-radius:11px;background:linear-gradient(140deg,var(--ac2),var(--ac));display:grid;place-items:center;color:#08130a;font-weight:800;font-size:16px;letter-spacing:-.5px;flex:0 0 auto}
.brand h1{margin:0;font-size:21px;font-weight:700;color:var(--txt);letter-spacing:.3px}
.brand span{display:block;font-size:11.5px;font-weight:400;color:var(--dim);text-transform:uppercase;letter-spacing:.8px}
.sub{margin:16px 0 20px;color:var(--dim);font-size:13.5px;line-height:1.5}
label{display:block;font-size:11.5px;font-weight:600;color:var(--faint);text-transform:uppercase;letter-spacing:.9px;margin:14px 0 6px}
input{width:100%;background:var(--field);color:var(--txt);border:1px solid var(--b);border-radius:10px;padding:13px;font-size:15.5px;font-family:inherit;min-height:46px}
input:focus{outline:0;border-color:var(--ac2)}
button{width:100%;margin-top:22px;background:var(--ac);color:#fff;border:0;border-radius:10px;padding:14px;font-size:15.5px;font-weight:600;font-family:inherit;cursor:pointer;min-height:48px}
button:hover{filter:brightness(1.14)}
button:active{transform:translateY(1px)}
#m{min-height:20px;margin:14px 0 0;font-size:13.5px;color:#ff7b72;text-align:center}
.rodape{margin-top:22px;padding-top:16px;border-top:1px solid var(--b);color:var(--faint);font-size:12px;text-align:center;line-height:1.6}
</style></head><body>
<div class="box">
<div class="brand"><div class="mark">SMI</div><div><h1>SMI DNC</h1><span>Transferencia CNC</span></div></div>
<p class="sub">Entre para configurar este equipamento e comandar as transferencias com a maquina.</p>
<label for="u">Usuario</label>
<input id="u" autocomplete="username" autocapitalize="none" autocorrect="off" spellcheck="false" placeholder="admin" autofocus>
<label for="p">Senha</label>
<input id="p" type="password" autocomplete="current-password" placeholder="Senha do equipamento">
<button onclick="go()">Entrar</button>
<p id="m"></p>
<div class="rodape">SMI Automacao Industrial<br>Acesso restrito a pessoal autorizado</div>
</div>
<script>
async function go(){m.textContent='';let r=await fetch('/api/login',{method:'POST',body:new URLSearchParams({user:u.value,pass:p.value})});if(r.ok)location='/';else m.textContent='Usuario ou senha invalidos.';}
// Enter envia. Antes so o clique funcionava, e num celular o teclado mostra
// justamente a tecla de confirmar -- o operador apertava e nada acontecia.
document.addEventListener('keydown',e=>{if(e.key==='Enter')go()});
</script></body></html>
)HTML";

WebPortal::WebPortal() : _server(HTTP_PORT) {}

void WebPortal::begin() {
  const char* headers[] = {"Cookie"};
  _server.collectHeaders(headers, 1);
  registerRoutes();
  _server.begin();
  logEvent("INFO", "Servidor web iniciado");
}

void WebPortal::loop() {
  _server.handleClient();
}

String WebPortal::randomToken() {
  String t;
  for (int i = 0; i < 32; i++) t += String((uint8_t)esp_random(), HEX);
  return t;
}

bool WebPortal::authenticated(uint8_t minRole) {
  if (_sessionToken.length() == 0) return false;
  String cookie = _server.header("Cookie");
  bool has = cookie.indexOf("SID=" + _sessionToken) >= 0;
  return has && _sessionRole >= minRole;
}

bool WebPortal::requireAuth(uint8_t minRole) {
  if (authenticated(minRole)) return true;
  sendJson(401, "{\"ok\":false,\"error\":\"unauthorized\"}");
  return false;
}

void WebPortal::sendJson(int code, const String& json) {
  _server.sendHeader("Cache-Control", "no-store");
  _server.send(code, "application/json", json);
}

void WebPortal::sendPlain(int code, const String& text) {
  _server.send(code, "text/plain; charset=utf-8", text);
}

void WebPortal::registerRoutes() {
  _server.on("/", HTTP_GET, [this]() { handleRoot(); });
  _server.on("/login", HTTP_GET, [this]() { handleLoginPage(); });
  _server.on("/api/login", HTTP_POST, [this]() { handleLoginApi(); });
  _server.on("/logout", HTTP_GET, [this]() { handleLogout(); });
  _server.on("/api/status", HTTP_GET, [this]() { handleStatus(); });
  _server.on("/api/files", HTTP_GET, [this]() { handleFiles(); });
  _server.on("/api/mkdir", HTTP_POST, [this]() { handleMkdir(); });
  _server.on("/api/delete", HTTP_POST, [this]() { handleDelete(); });
  _server.on("/api/rename", HTTP_POST, [this]() { handleRename(); });
  _server.on("/api/copy", HTTP_POST, [this]() { handleCopy(); });
  _server.on("/api/move", HTTP_POST, [this]() { handleMove(); });
  _server.on("/download", HTTP_GET, [this]() { handleDownload(); });
  _server.on("/api/upload", HTTP_POST, [this]() { handleUploadDone(); }, [this]() { handleUpload(); });
  _server.on("/api/cmd", HTTP_POST, [this]() { handleCommand(); });
  _server.on("/api/serial", HTTP_GET, [this]() { handleSerialGet(); });
  _server.on("/api/serial", HTTP_POST, [this]() { handleSerialSet(); });
  _server.on("/api/network", HTTP_GET, [this]() { handleNetworkGet(); });
  _server.on("/api/network", HTTP_POST, [this]() { handleNetworkSet(); });
  _server.on("/api/senhas", HTTP_POST, [this]() { handleSenhasSet(); });
  _server.on("/api/logs", HTTP_GET, [this]() { handleLogs(); });
  _server.onNotFound([this]() { sendPlain(404, "Nao encontrado"); });
}

void WebPortal::handleRoot() {
  if (!authenticated()) { _server.sendHeader("Location", "/login"); _server.send(302, "text/plain", ""); return; }
  _server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void WebPortal::handleLoginPage() {
  _server.send_P(200, "text/html; charset=utf-8", LOGIN_HTML);
}

void WebPortal::handleLoginApi() {
  String u = _server.arg("user");
  String p = _server.arg("pass");
  // As senhas vem da memoria do equipamento, nao mais do codigo. Ver a nota em
  // AccessConfig: senha igual em todo aparelho e uma chave so para toda a
  // frota.
  uint8_t role = 0;
  if (u == DEFAULT_ADMIN_USER && p == gAccessConfig.adminPass) role = 3;
  else if (u == DEFAULT_OPER_USER && p == gAccessConfig.operPass) role = 2;
  else if (u == DEFAULT_VIEW_USER && p == gAccessConfig.viewPass) role = 1;
  if (!role) { sendJson(403, "{\"ok\":false}"); return; }
  _sessionRole = role;
  _sessionToken = randomToken();
  _server.sendHeader("Set-Cookie", "SID=" + _sessionToken + "; Path=/; SameSite=Strict");
  sendJson(200, "{\"ok\":true}");
}

void WebPortal::handleLogout() {
  _sessionToken = "";
  _sessionRole = 0;
  _server.sendHeader("Set-Cookie", "SID=; Path=/; Max-Age=0");
  _server.sendHeader("Location", "/login");
  _server.send(302, "text/plain", "");
}

void WebPortal::handleStatus() {
  if (!requireAuth()) return;
  SystemStatus s;
  xSemaphoreTake(gStatusMutex, portMAX_DELAY);
  memcpy(&s, &gStatus, sizeof(s));
  xSemaphoreGive(gStatusMutex);
  String ip = WiFi.isConnected() ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  String json;

  // RESERVAR ANTES DE CONCATENAR.
  //
  // Mesmo problema do log, em escala menor: sem reserva, cada `+=` pode
  // realocar e copiar. Esta resposta e consultada em laco pela tela de estado,
  // entao o custo se repete o tempo todo.
  json.reserve(640);
  json = "{";
  json += "\"state\":" + String((int)s.state) + ",\"stateText\":\"" + escapeJson(stateToText(s.state)) + "\",";
  json += "\"serialConnected\":" + String(s.serialConnected ? "true" : "false") + ",";
  json += "\"sdOk\":" + String(s.sdOk ? "true" : "false") + ",\"wifiOk\":" + String(s.wifiOk ? "true" : "false") + ",\"apMode\":" + String(s.apMode ? "true" : "false") + ",";
  json += "\"flowPaused\":" + String(s.flowPaused ? "true" : "false") + ",\"ctsOk\":" + String(s.ctsOk ? "true" : "false") + ",\"dsrOk\":" + String(s.dsrOk ? "true" : "false") + ",";
  json += "\"currentFile\":\"" + escapeJson(String(s.currentFile)) + "\",\"fileSize\":" + String(s.fileSize) + ",\"bytesSent\":" + String(s.bytesSent) + ",\"bytesReceived\":" + String(s.bytesReceived) + ",\"bps\":" + String(s.bytesPerSecond) + ",";
  json += "\"flowText\":\"" + escapeJson(flowToText(gSerialConfig.flow)) + "\",\"freeBytes\":" + String(storage.freeBytes()) + ",\"totalBytes\":" + String(storage.totalBytes()) + ",\"ip\":\"" + ip + "\",";
  json += "\"versao\":\"" FIRMWARE_VERSION "\",\"build\":\"" FIRMWARE_BUILD "\",";
  json += "\"particao\":\"" + escapeJson(particaoEmUso()) + "\"}";
  sendJson(200, json);
}

void WebPortal::handleFiles() {
  if (!requireAuth()) return;
  String p = _server.hasArg("path") ? _server.arg("path") : "/";
  sendJson(200, storage.listJson(p));
}

void WebPortal::handleMkdir() {
  if (!requireAuth(2)) return;
  bool ok = storage.makeDir(_server.arg("path"));
  sendJson(200, ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"mkdir failed\"}");
}

void WebPortal::handleDelete() {
  if (!requireAuth(2)) return;
  bool ok = storage.removePath(_server.arg("path"));
  sendJson(200, ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"delete failed\"}");
}

void WebPortal::handleRename() {
  if (!requireAuth(2)) return;
  bool ok = storage.renamePath(_server.arg("from"), _server.arg("to"));
  sendJson(200, ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"rename failed\"}");
}

void WebPortal::handleCopy() {
  if (!requireAuth(2)) return;
  bool ok = storage.copyFile(_server.arg("from"), _server.arg("to"));
  sendJson(200, ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"copy failed\"}");
}

void WebPortal::handleMove() {
  if (!requireAuth(2)) return;
  bool ok = storage.movePath(_server.arg("from"), _server.arg("to"));
  sendJson(200, ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"move failed\"}");
}

void WebPortal::handleDownload() {
  if (!requireAuth()) return;
  String p = storage.sanitizePath(_server.arg("path"));
  xSemaphoreTake(storage.mutex(), portMAX_DELAY);
  File f = SD.open(p, FILE_READ);
  if (!f || f.isDirectory()) {
    if (f) f.close();
    xSemaphoreGive(storage.mutex());
    sendPlain(404, "Arquivo nao encontrado");
    return;
  }
  String name = p.substring(p.lastIndexOf('/') + 1);
  _server.sendHeader("Content-Disposition", "attachment; filename=\"" + name + "\"");
  _server.streamFile(f, "application/octet-stream");
  f.close();
  xSemaphoreGive(storage.mutex());
}

void WebPortal::handleUpload() {
  if (!authenticated(2)) return;
  HTTPUpload& up = _server.upload();
  if (up.status == UPLOAD_FILE_START) {
    String dir = _server.arg("dir");
    if (dir.length() == 0) dir = DIR_PROGRAMS;

    // O nome vem do cabecalho do formulario e NAO e confiavel -- nem por
    // maldade, nem por acaso: o proprio analisador do WebServer o entrega
    // contaminado quando ha `filename*=`. Ver sanitizeFileName().
    const String nome = storage.sanitizeFileName(up.filename);
    _uploadPath = storage.sanitizePath(dir + "/" + nome);
    logEventf("INFO", "Upload recebendo: %s", _uploadPath.c_str());
    xSemaphoreTake(storage.mutex(), portMAX_DELAY);
    _uploadFile = SD.open(_uploadPath, FILE_WRITE);
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (_uploadFile) _uploadFile.write(up.buf, up.currentSize);
  } else if (up.status == UPLOAD_FILE_END) {
    if (_uploadFile) _uploadFile.close();
    xSemaphoreGive(storage.mutex());
    logEventf("INFO", "Upload concluido: %s", _uploadPath.c_str());
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (_uploadFile) _uploadFile.close();
    xSemaphoreGive(storage.mutex());
    logEvent("ERRO", "Upload abortado");
  }
}

void WebPortal::handleUploadDone() {
  if (!requireAuth(2)) return;
  sendPlain(200, "Upload concluido");
}

void WebPortal::handleCommand() {
  if (!requireAuth(2)) return;
  String a = _server.arg("action");

  // Pausa, retomada e cancelamento precisam agir mesmo quando TaskDNC esta dentro
  // de uma transferencia longa. Por isso estes comandos alteram flags diretamente.
  if (a == "pause") {
    xSemaphoreTake(gStatusMutex, portMAX_DELAY);
    gStatus.userPaused = true;
    gStatus.state = DNC_PAUSED_USER;
    xSemaphoreGive(gStatusMutex);
    logEvent("INFO", "Pausa solicitada pelo operador");
    sendJson(200, "{\"ok\":true}");
    return;
  }
  if (a == "resume") {
    xSemaphoreTake(gStatusMutex, portMAX_DELAY);
    gStatus.userPaused = false;
    if (gStatus.serialConnected) gStatus.state = DNC_CONNECTED;
    xSemaphoreGive(gStatusMutex);
    logEvent("INFO", "Retomada solicitada pelo operador");
    sendJson(200, "{\"ok\":true}");
    return;
  }
  // CONFIRMAR O FIM E DECISAO DO OPERADOR, nunca do relogio.
  //
  // O equipamento para em "transferido, aguardando confirmacao" porque o
  // programa ter saido pela serial NAO significa que a maquina terminou de
  // executar. Fechar a comunicacao antes disso pode cortar o final do programa.
  if (a == "finish") {
    finalizarSessaoPeloOperador();
    logEvent("INFO", "Encerramento confirmado pelo operador");
    sendJson(200, "{\"ok\":true}");
    return;
  }

  // "Continuar aguardando" na pergunta de drenagem travada. Reabre o prazo; sem
  // isto a pergunta voltaria no instante seguinte, num laco sem saida.
  if (a == "continue") {
    continuarAguardandoDrenagem();
    logEvent("INFO", "Operador optou por continuar aguardando a maquina");
    sendJson(200, "{\"ok\":true}");
    return;
  }

  if (a == "cancel") {
    xSemaphoreTake(gStatusMutex, portMAX_DELAY);
    gStatus.cancelRequested = true;
    gStatus.userPaused = false;
    gStatus.state = DNC_CANCELING;
    xSemaphoreGive(gStatusMutex);
    txRing.clear();
    logEvent("WARN", "Cancelamento solicitado pelo operador");
    sendJson(200, "{\"ok\":true}");
    return;
  }

  DncCommand cmd;
  if (a == "connect") cmd.type = CMD_CONNECT;
  else if (a == "disconnect") cmd.type = CMD_DISCONNECT;
  else if (a == "send") cmd.type = CMD_SEND_FILE;
  else if (a == "receive") cmd.type = CMD_RECEIVE_FILE;
  else if (a == "drip") cmd.type = CMD_DRIP_FILE;
  else { sendJson(400, "{\"ok\":false,\"error\":\"acao invalida\"}"); return; }

  String p = storage.sanitizePath(_server.arg("path"));
  strncpy(cmd.path, p.c_str(), sizeof(cmd.path) - 1);
  bool ok = xQueueSend(gDncCommandQueue, &cmd, pdMS_TO_TICKS(50)) == pdTRUE;
  sendJson(200, ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"fila cheia\"}");
}

void WebPortal::handleSerialGet() {
  if (!requireAuth()) return;
  String json = "{";
  json += "\"baudRate\":" + String(gSerialConfig.baudRate) + ",\"dataBits\":" + String(gSerialConfig.dataBits) + ",\"parity\":" + String((int)gSerialConfig.parity) + ",\"stopBits\":" + String(gSerialConfig.stopBits) + ",\"timeoutMs\":" + String(gSerialConfig.timeoutMs) + ",\"flow\":" + String((int)gSerialConfig.flow) + "}";
  sendJson(200, json);
}

void WebPortal::handleSerialSet() {
  if (!requireAuth(3)) return;
  gSerialConfig.baudRate = _server.arg("baudRate").toInt();
  gSerialConfig.dataBits = _server.arg("dataBits").toInt();
  gSerialConfig.parity = (ParityMode)_server.arg("parity").toInt();
  gSerialConfig.stopBits = _server.arg("stopBits").toInt();
  gSerialConfig.timeoutMs = _server.arg("timeoutMs").toInt();
  gSerialConfig.flow = (FlowControlMode)_server.arg("flow").toInt();
  settings.saveSerial(gSerialConfig);
  DncCommand cmd;
  cmd.type = CMD_APPLY_SERIAL;
  xQueueSend(gDncCommandQueue, &cmd, pdMS_TO_TICKS(50));
  sendJson(200, "{\"ok\":true}");
}

void WebPortal::handleNetworkGet() {
  if (!requireAuth()) return;
  String json = "{";
  json += "\"nome\":\"" + escapeJson(gNetworkConfig.nomeDoEquipamento) + "\",";
  json += "\"ssid\":\"" + escapeJson(gNetworkConfig.ssid) + "\",\"dhcp\":" + String(gNetworkConfig.dhcp ? "true" : "false") + ",";
  json += "\"ip\":\"" + gNetworkConfig.ip.toString() + "\",\"gateway\":\"" + gNetworkConfig.gateway.toString() + "\",\"subnet\":\"" + gNetworkConfig.subnet.toString() + "\",\"dns\":\"" + gNetworkConfig.dns.toString() + "\",";
  // As senhas NAO saem daqui. Nem a do Wi-Fi nem a do OTA: quem consegue abrir
  // esta pagina passaria a ter as duas. Devolvemos so se existe ou nao.
  json += "\"otaSenhaDefinida\":" + String(gNetworkConfig.otaPassword.length() ? "true" : "false") + ",";
  json += "\"otaAtivo\":" + String(gOta.iniciado() ? "true" : "false") + ",";
  json += "\"otaNome\":\"" + escapeJson(gNetworkConfig.nomeDoEquipamento) + "\",";
  // So o FATO de a senha ser a de fabrica, nunca a senha.
  json += "\"senhaDeFabrica\":" + String(settings.usandoSenhasDeFabrica(gAccessConfig) ? "true" : "false") + "}";
  sendJson(200, json);
}

void WebPortal::handleNetworkSet() {
  if (!requireAuth(3)) return;
  // O nome passa pelo saneamento ANTES de ser gravado, e o operador ve na tela
  // o que de fato ficou. Nome invalido nao da erro: o mDNS so nao anuncia e o
  // DHCP ignora -- falha silenciosa, a pior de diagnosticar.
  const String nomePedido = _server.arg("nome");
  if (nomePedido.length()) {
    gNetworkConfig.nomeDoEquipamento = Settings::sanitizarNomeDeEquipamento(nomePedido);
  }
  // MESMA REGRA DOS VIZINHOS: CAMPO EM BRANCO E "NAO MEXER", NAO "APAGAR".
  //
  // Este era o unico campo da tela sem essa guarda -- e o unico cuja perda tira
  // o equipamento da rede inteira. Salvar a tela para mudar OUTRA coisa (o
  // DHCP, por exemplo) com o campo de rede em branco apagava o SSID, e a caixa
  // sumia. Quem estiver no cliente nao vai relacionar uma coisa com a outra.
  //
  // Para trocar de rede continua bastando digitar a nova: so o branco e que
  // deixou de ter efeito destrutivo.
  const String ssidPedido = _server.arg("ssid");
  if (ssidPedido.length()) gNetworkConfig.ssid = ssidPedido;

  String pass = _server.arg("password");
  if (pass.length()) gNetworkConfig.password = pass;
  // Mesma regra da senha de Wi-Fi: campo em branco significa "nao mexer", nao
  // "apagar". Sem isto, salvar a rede desligaria o OTA sem querer.
  String otaPass = _server.arg("otapass");
  if (otaPass.length()) gNetworkConfig.otaPassword = otaPass;
  gNetworkConfig.dhcp = _server.arg("dhcp") != "0";
  gNetworkConfig.ip.fromString(_server.arg("ip"));
  gNetworkConfig.gateway.fromString(_server.arg("gateway"));
  gNetworkConfig.subnet.fromString(_server.arg("subnet"));
  gNetworkConfig.dns.fromString(_server.arg("dns"));
  settings.saveNetwork(gNetworkConfig);
  sendJson(200, "{\"ok\":true}");
}

void WebPortal::handleSenhasSet() {
  if (!requireAuth(3)) return;

  const String adm  = _server.arg("admin");
  const String ope  = _server.arg("operador");
  const String view = _server.arg("viewer");

  // CAMPO EM BRANCO SIGNIFICA "NAO MEXER", nunca "apagar".
  //
  // Mesma regra das senhas de Wi-Fi e de atualizacao. Sem ela, salvar para
  // trocar so a do operador zeraria as outras duas -- e a do administrador em
  // branco trancaria o equipamento para fora.
  int trocadas = 0;
  AccessConfig nova = gAccessConfig;

  // Minimo de 4 caracteres. Nao e forca de senha: e impedir o dedo escorregado
  // que salva uma senha de um caractere sem perceber.
  if (adm.length())  { if (adm.length()  < 4) { sendJson(400, "{\"ok\":false,\"error\":\"senha de administrador muito curta\"}"); return; } nova.adminPass = adm; ++trocadas; }
  if (ope.length())  { if (ope.length()  < 4) { sendJson(400, "{\"ok\":false,\"error\":\"senha de operador muito curta\"}"); return; }      nova.operPass  = ope; ++trocadas; }
  if (view.length()) { if (view.length() < 4) { sendJson(400, "{\"ok\":false,\"error\":\"senha de consulta muito curta\"}"); return; }      nova.viewPass  = view; ++trocadas; }

  if (trocadas == 0) { sendJson(400, "{\"ok\":false,\"error\":\"nenhuma senha informada\"}"); return; }

  gAccessConfig = nova;
  settings.saveAccess(gAccessConfig);

  // A sessao aberta NAO cai. Quem trocou a senha ja provou quem e; derrubar o
  // proprio administrador no meio da configuracao so atrapalha.
  logEventf("INFO", "%d senha(s) de acesso alterada(s)", trocadas);
  sendJson(200, "{\"ok\":true}");
}

void WebPortal::handleLogs() {
  if (!requireAuth()) return;

  // ENVIA EM PEDACOS. NAO MONTE O JSON INTEIRO NA RAM.
  //
  // MEDIDO EM BANCADA: a versao anterior montava tudo num unico `String` com
  // `+=` linha a linha. O texto final tem ~10 KB, mas o PICO de memoria era de
  // ~55 KB por requisicao -- a escada de realocacao do String (a cada
  // crescimento, a copia velha e a nova coexistem) mais as copias intermediarias
  // ate o socket.
  //
  // Numa caixa com ~100 KB livres, isso significa que DUAS consultas ao log
  // durante uma transferencia esgotam a memoria. Foi exatamente o que
  // aconteceu: o mDNS comecou a falhar com "free heap: 104 bytes".
  //
  // Mandando em pedacos, o pico passa a ser o de UMA linha.
  _server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  _server.send(200, "application/json; charset=utf-8", "");
  _server.sendContent("{\"lines\":[");

  // O MUTEX NAO PODE ATRAVESSAR UMA ESCRITA DE REDE, E O CAO PRECISA COMER.
  //
  // MEDIDO NA POLARIS: a caixa reiniciou varias vezes com WATCHDOG DE TAREFA.
  // Causa: `sendContent` BLOQUEIA quando a janela TCP enche -- e com Wi-Fi a
  // -74 dBm isso acontece a toda hora. A TaskWebServer fica presa dentro de
  // handleClient() e so alimenta o cao quando VOLTA ao laco principal; 200
  // linhas de log nessas condicoes passam dos 15 s e o equipamento reinicia.
  //
  // E o mesmo padrao ja corrigido em streamFileToCnc, repetido aqui.
  //
  // Duas coisas mudam:
  //   1. o cao e alimentado a cada linha, dentro da requisicao;
  //   2. o mutex e tomado SO para copiar a linha, nunca durante o envio -- com
  //      ele preso, a TaskLogger tambem ficava esperando a rede.
  bool first = true;
  for (uint16_t i = 0; i < MAX_RECENT_LOGS; i++) {
    uint16_t idx = (gRecentLogHead + i) % MAX_RECENT_LOGS;

    String linha;
    xSemaphoreTake(gLogMutex, portMAX_DELAY);
    if (gRecentLogs[idx].length() > 0) linha = gRecentLogs[idx];
    xSemaphoreGive(gLogMutex);
    if (linha.length() == 0) continue;

    String saida = first ? String("\"") : String(",\"");
    first = false;
    saida += escapeJson(linha);
    saida += "\"";
    _server.sendContent(saida);
    esp_task_wdt_reset();
  }

  _server.sendContent("]}");
  _server.sendContent("");   // fecha a resposta em pedacos
}
