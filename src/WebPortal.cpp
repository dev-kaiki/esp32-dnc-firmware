#include "WebPortal.h"
#include <WiFi.h>
#include "Logger.h"
#include "../Config.h"
#include "CircularBuffer.h"

extern QueueHandle_t gDncCommandQueue;
extern CircularBuffer txRing;
extern SystemStatus gStatus;
extern SemaphoreHandle_t gStatusMutex;
extern DncSerialConfig gSerialConfig;
extern NetworkConfig gNetworkConfig;
extern Settings settings;
extern StorageManager storage;
extern String gRecentLogs[MAX_RECENT_LOGS];
extern uint16_t gRecentLogHead;
extern SemaphoreHandle_t gLogMutex;

WebPortal webPortal;

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="pt-BR"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SMI ESP32 DNC</title><style>
:root{--g:#00a86b;--bg:#0f1720;--panel:#17212b;--txt:#eef3f7;--mut:#a9b4bf;--red:#e74c3c;--yel:#f1c40f;--b:#263544}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--txt);font-family:Arial,Helvetica,sans-serif}.top{background:#071018;border-bottom:3px solid var(--g);padding:12px 18px;display:flex;justify-content:space-between;align-items:center}.brand{font-weight:700;font-size:20px}.brand span{color:var(--g)}.wrap{display:grid;grid-template-columns:220px 1fr;min-height:calc(100vh - 55px)}nav{background:#111b24;padding:14px;border-right:1px solid #223040}nav button{width:100%;margin:4px 0;padding:12px;border:0;border-radius:8px;background:#1c2a36;color:var(--txt);text-align:left;cursor:pointer}nav button:hover,nav button.active{background:var(--g);color:#001a10}main{padding:18px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px}.card{background:var(--panel);border:1px solid var(--b);border-radius:10px;padding:14px}.card h3{margin:0 0 8px}.value{font-size:20px;font-weight:700}.muted{color:var(--mut)}.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}button,.btn{background:var(--g);color:#001a10;border:0;border-radius:8px;padding:10px 12px;font-weight:700;cursor:pointer}button.warn{background:var(--yel)}button.danger{background:var(--red);color:#fff}button.gray{background:#314253;color:#fff}input,select{background:#0d151d;color:var(--txt);border:1px solid #34485d;border-radius:8px;padding:10px}table{width:100%;border-collapse:collapse;background:var(--panel);border-radius:10px;overflow:hidden}td,th{border-bottom:1px solid #263544;padding:9px;text-align:left}progress{width:100%;height:18px}.ok{color:#42f58d}.bad{color:#ff6961}.log{height:360px;overflow:auto;background:#061018;border:1px solid #263544;border-radius:8px;padding:10px;font-family:Consolas,monospace;font-size:13px;white-space:pre-wrap}.hidden{display:none}@media(max-width:800px){.wrap{grid-template-columns:1fr}nav{display:flex;overflow:auto}nav button{min-width:130px}}
</style></head><body><div class="top"><div class="brand"><span>SMI</span> ESP32 DNC Industrial</div><button class="gray" onclick="logout()">Sair</button></div><div class="wrap"><nav>
<button onclick="tab('dash')" class="active">Dashboard</button><button onclick="tab('files')">Arquivos</button><button onclick="tab('transfer')">Transferência</button><button onclick="tab('serial')">Serial RS232</button><button onclick="tab('network')">Rede</button><button onclick="tab('logs')">Logs</button></nav><main>
<section id="dash"><div class="grid"><div class="card"><h3>Estado</h3><div class="value" id="stState">-</div><div class="muted" id="stFile">-</div></div><div class="card"><h3>RS232</h3><div id="stSerial">-</div><div id="stFlow">-</div></div><div class="card"><h3>microSD</h3><div id="stSd">-</div><div id="stSpace">-</div></div><div class="card"><h3>Rede</h3><div id="stNet">-</div><div class="muted" id="stIp">-</div></div></div><div class="card"><h3>Progresso</h3><progress id="prog" value="0" max="100"></progress><div id="stProg">0%</div></div></section>
<section id="files" class="hidden"><h2>Gerenciador de Arquivos</h2><div class="row"><button onclick="upDir()">Voltar</button><input id="cwd" value="/programs"><button onclick="loadFiles()">Abrir</button><input id="mkdirName" placeholder="Nova pasta"><button onclick="mkdir()">Criar pasta</button></div><div class="row"><input type="file" id="up"><button onclick="upload()">Upload</button></div><table><thead><tr><th>Nome</th><th>Tipo</th><th>Tamanho</th><th>Ações</th></tr></thead><tbody id="fileRows"></tbody></table></section>
<section id="transfer" class="hidden"><h2>Transferência / DNC Online</h2><div class="card"><div class="row"><input id="cmdPath" style="min-width:320px" placeholder="/programs/arquivo.nc"><button onclick="cmd('connect')">Conectar</button><button class="gray" onclick="cmd('disconnect')">Desconectar</button></div><div class="row"><button onclick="cmd('send')">Enviar para CNC</button><button onclick="cmd('receive')">Receber da CNC</button><button onclick="cmd('drip')">Executar Online</button><button class="warn" onclick="cmd('pause')">Pausar</button><button onclick="cmd('resume')">Retomar</button><button class="danger" onclick="cmd('cancel')">Cancelar</button></div><p class="muted">Para receber, informe opcionalmente o caminho de destino. Se vazio, será salvo em /received automaticamente.</p></div></section>
<section id="serial" class="hidden"><h2>Configuração Serial RS232</h2><div class="grid"><div class="card"><label>Baud Rate</label><input id="baud" value="9600"></div><div class="card"><label>Data Bits</label><select id="dataBits"><option>5</option><option>6</option><option selected>7</option><option>8</option></select></div><div class="card"><label>Paridade</label><select id="parity"><option value="0">NONE</option><option value="1" selected>EVEN</option><option value="2">ODD</option></select></div><div class="card"><label>Stop Bits</label><select id="stopBits"><option selected>1</option><option>2</option></select></div><div class="card"><label>Timeout ms</label><input id="timeout" value="3000"></div><div class="card"><label>Controle de Fluxo</label><select id="flow"><option value="0">NONE</option><option value="1" selected>XON/XOFF</option><option value="2">RTS/CTS</option><option value="3">DTR/DSR</option></select></div></div><div class="row"><button onclick="saveSerial()">Salvar e aplicar</button></div></section>
<section id="network" class="hidden"><h2>Configuração de Rede</h2><div class="grid"><div class="card"><label>SSID</label><input id="ssid"></div><div class="card"><label>Senha Wi-Fi</label><input id="wpass" type="password"></div><div class="card"><label>DHCP</label><select id="dhcp"><option value="1">Sim</option><option value="0">IP Fixo</option></select></div><div class="card"><label>IP</label><input id="ip" placeholder="192.168.1.220"></div><div class="card"><label>Gateway</label><input id="gw" placeholder="192.168.1.1"></div><div class="card"><label>Máscara</label><input id="mask" placeholder="255.255.255.0"></div><div class="card"><label>DNS</label><input id="dns" placeholder="8.8.8.8"></div></div><div class="row"><button onclick="saveNetwork()">Salvar rede</button></div><p class="muted">Após alterar rede, reinicie o equipamento.</p></section>
<section id="logs" class="hidden"><h2>Logs Operacionais</h2><div class="row"><button onclick="loadLogs()">Atualizar</button><a class="btn" href="/download?path=/logs/dnc.log">Baixar log</a></div><div id="logBox" class="log"></div></section>
</main></div><script>
let current='/programs';function qs(id){return document.getElementById(id)}function tab(id){document.querySelectorAll('main section').forEach(s=>s.classList.add('hidden'));qs(id).classList.remove('hidden');document.querySelectorAll('nav button').forEach(b=>b.classList.remove('active'));event.target.classList.add('active');if(id==='files')loadFiles();if(id==='serial')loadSerial();if(id==='network')loadNetwork();if(id==='logs')loadLogs()}
async function get(u){let r=await fetch(u);if(r.status===401)location='/login';return r.json()}async function post(u,d){let r=await fetch(u,{method:'POST',body:new URLSearchParams(d)});if(r.status===401)location='/login';return r.json()}
function fmt(n){n=Number(n||0);if(n>1073741824)return(n/1073741824).toFixed(2)+' GB';if(n>1048576)return(n/1048576).toFixed(2)+' MB';if(n>1024)return(n/1024).toFixed(1)+' KB';return n+' B'}
async function loadStatus(){let s=await get('/api/status');qs('stState').textContent=s.stateText;qs('stFile').textContent=s.currentFile||'-';qs('stSerial').innerHTML=s.serialConnected?'<span class="ok">Conectado</span>':'<span class="bad">Desconectado</span>';qs('stFlow').textContent='Fluxo: '+s.flowText+' | CTS '+(s.ctsOk?'OK':'BLOQ')+' | DSR '+(s.dsrOk?'OK':'BLOQ')+(s.flowPaused?' | PAUSADO':'');qs('stSd').innerHTML=s.sdOk?'<span class="ok">OK</span>':'<span class="bad">Falha</span>';qs('stSpace').textContent='Livre: '+fmt(s.freeBytes)+' / Total: '+fmt(s.totalBytes);qs('stNet').innerHTML=s.wifiOk?'<span class="ok">Online</span>':(s.apMode?'AP Local':'Offline');qs('stIp').textContent=s.ip||'-';let pct=s.fileSize?Math.floor((s.bytesSent*100)/s.fileSize):0;qs('prog').value=pct;qs('stProg').textContent=pct+'% | Enviado '+fmt(s.bytesSent)+' / '+fmt(s.fileSize)+' | RX '+fmt(s.bytesReceived)+' | '+fmt(s.bps)+'/s'}
async function loadFiles(){current=qs('cwd').value||current;let j=await get('/api/files?path='+encodeURIComponent(current));current=j.path;qs('cwd').value=current;let rows='';j.items.forEach(f=>{rows+=`<tr><td>${f.dir?'📁':'📄'} ${f.name}</td><td>${f.dir?'Pasta':'Arquivo'}</td><td>${fmt(f.size)}</td><td><button onclick="openItem('${f.path}',${f.dir})">Abrir</button> <button onclick="pick('${f.path}')">Selecionar</button> <button class="gray" onclick="ren('${f.path}')">Renomear</button> <button class="danger" onclick="delp('${f.path}')">Excluir</button> <a class="btn" href="/download?path=${encodeURIComponent(f.path)}">Download</a></td></tr>`});qs('fileRows').innerHTML=rows}
function openItem(p,d){if(d){current=p;qs('cwd').value=p;loadFiles()}}function pick(p){qs('cmdPath').value=p;tabDirect('transfer')}function tabDirect(id){document.querySelectorAll('main section').forEach(s=>s.classList.add('hidden'));qs(id).classList.remove('hidden')}function upDir(){let p=current.split('/');p.pop();current=p.join('/')||'/';qs('cwd').value=current;loadFiles()}async function mkdir(){await post('/api/mkdir',{path:current+'/'+qs('mkdirName').value});loadFiles()}async function delp(p){if(confirm('Excluir '+p+'?')){await post('/api/delete',{path:p});loadFiles()}}async function ren(p){let n=prompt('Novo caminho completo:',p);if(n)await post('/api/rename',{from:p,to:n});loadFiles()}async function upload(){let f=qs('up').files[0];if(!f)return alert('Selecione arquivo');let fd=new FormData();fd.append('dir',current);fd.append('file',f);let r=await fetch('/api/upload',{method:'POST',body:fd});alert(await r.text());loadFiles()}
async function cmd(a){let p=qs('cmdPath').value;let j=await post('/api/cmd',{action:a,path:p});alert(j.ok?'Comando aceito':j.error)}
async function loadSerial(){let s=await get('/api/serial');qs('baud').value=s.baudRate;qs('dataBits').value=s.dataBits;qs('parity').value=s.parity;qs('stopBits').value=s.stopBits;qs('timeout').value=s.timeoutMs;qs('flow').value=s.flow}async function saveSerial(){let j=await post('/api/serial',{baudRate:qs('baud').value,dataBits:qs('dataBits').value,parity:qs('parity').value,stopBits:qs('stopBits').value,timeoutMs:qs('timeout').value,flow:qs('flow').value});alert(j.ok?'Serial aplicado':j.error)}
async function loadNetwork(){let n=await get('/api/network');qs('ssid').value=n.ssid;qs('dhcp').value=n.dhcp?'1':'0';qs('ip').value=n.ip;qs('gw').value=n.gateway;qs('mask').value=n.subnet;qs('dns').value=n.dns}async function saveNetwork(){let j=await post('/api/network',{ssid:qs('ssid').value,password:qs('wpass').value,dhcp:qs('dhcp').value,ip:qs('ip').value,gateway:qs('gw').value,subnet:qs('mask').value,dns:qs('dns').value});alert(j.ok?'Rede salva. Reinicie.':j.error)}
async function loadLogs(){let l=await get('/api/logs');qs('logBox').textContent=l.lines.join('\n');qs('logBox').scrollTop=qs('logBox').scrollHeight}function logout(){location='/logout'}setInterval(loadStatus,1000);loadStatus();
</script></body></html>
)HTML";

static const char LOGIN_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="pt-BR"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Login SMI DNC</title><style>body{margin:0;background:#0f1720;color:#eef3f7;font-family:Arial;display:grid;place-items:center;height:100vh}.box{width:min(380px,92vw);background:#17212b;border:1px solid #263544;border-radius:12px;padding:24px}.logo{font-size:24px;font-weight:700}.logo span{color:#00a86b}input,button{width:100%;padding:12px;margin:8px 0;border-radius:8px;border:1px solid #34485d;background:#0d151d;color:#fff}button{background:#00a86b;color:#001a10;font-weight:700}</style></head><body><div class="box"><div class="logo"><span>SMI</span> ESP32 DNC</div><p>Login do equipamento</p><input id="u" placeholder="Usuário"><input id="p" type="password" placeholder="Senha"><button onclick="go()">Entrar</button><pre id="m"></pre></div><script>async function go(){let r=await fetch('/api/login',{method:'POST',body:new URLSearchParams({user:u.value,pass:p.value})});if(r.ok)location='/';else m.textContent='Login inválido';}</script></body></html>
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
  uint8_t role = 0;
  if (u == DEFAULT_ADMIN_USER && p == DEFAULT_ADMIN_PASS) role = 3;
  else if (u == DEFAULT_OPER_USER && p == DEFAULT_OPER_PASS) role = 2;
  else if (u == DEFAULT_VIEW_USER && p == DEFAULT_VIEW_PASS) role = 1;
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
  String json = "{";
  json += "\"state\":" + String((int)s.state) + ",\"stateText\":\"" + escapeJson(stateToText(s.state)) + "\",";
  json += "\"serialConnected\":" + String(s.serialConnected ? "true" : "false") + ",";
  json += "\"sdOk\":" + String(s.sdOk ? "true" : "false") + ",\"wifiOk\":" + String(s.wifiOk ? "true" : "false") + ",\"apMode\":" + String(s.apMode ? "true" : "false") + ",";
  json += "\"flowPaused\":" + String(s.flowPaused ? "true" : "false") + ",\"ctsOk\":" + String(s.ctsOk ? "true" : "false") + ",\"dsrOk\":" + String(s.dsrOk ? "true" : "false") + ",";
  json += "\"currentFile\":\"" + escapeJson(String(s.currentFile)) + "\",\"fileSize\":" + String(s.fileSize) + ",\"bytesSent\":" + String(s.bytesSent) + ",\"bytesReceived\":" + String(s.bytesReceived) + ",\"bps\":" + String(s.bytesPerSecond) + ",";
  json += "\"flowText\":\"" + escapeJson(flowToText(gSerialConfig.flow)) + "\",\"freeBytes\":" + String(storage.freeBytes()) + ",\"totalBytes\":" + String(storage.totalBytes()) + ",\"ip\":\"" + ip + "\"}";
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
    _uploadPath = storage.sanitizePath(dir + "/" + up.filename);
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
  json += "\"ssid\":\"" + escapeJson(gNetworkConfig.ssid) + "\",\"dhcp\":" + String(gNetworkConfig.dhcp ? "true" : "false") + ",";
  json += "\"ip\":\"" + gNetworkConfig.ip.toString() + "\",\"gateway\":\"" + gNetworkConfig.gateway.toString() + "\",\"subnet\":\"" + gNetworkConfig.subnet.toString() + "\",\"dns\":\"" + gNetworkConfig.dns.toString() + "\"}";
  sendJson(200, json);
}

void WebPortal::handleNetworkSet() {
  if (!requireAuth(3)) return;
  gNetworkConfig.ssid = _server.arg("ssid");
  String pass = _server.arg("password");
  if (pass.length()) gNetworkConfig.password = pass;
  gNetworkConfig.dhcp = _server.arg("dhcp") != "0";
  gNetworkConfig.ip.fromString(_server.arg("ip"));
  gNetworkConfig.gateway.fromString(_server.arg("gateway"));
  gNetworkConfig.subnet.fromString(_server.arg("subnet"));
  gNetworkConfig.dns.fromString(_server.arg("dns"));
  settings.saveNetwork(gNetworkConfig);
  sendJson(200, "{\"ok\":true}");
}

void WebPortal::handleLogs() {
  if (!requireAuth()) return;
  String json = "{\"lines\":[";
  xSemaphoreTake(gLogMutex, portMAX_DELAY);
  bool first = true;
  for (uint16_t i = 0; i < MAX_RECENT_LOGS; i++) {
    uint16_t idx = (gRecentLogHead + i) % MAX_RECENT_LOGS;
    if (gRecentLogs[idx].length() == 0) continue;
    if (!first) json += ",";
    first = false;
    json += "\"" + escapeJson(gRecentLogs[idx]) + "\"";
  }
  xSemaphoreGive(gLogMutex);
  json += "]}";
  sendJson(200, json);
}
