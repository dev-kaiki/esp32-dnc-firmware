#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include "Types.h"
#include "Settings.h"
#include "StorageManager.h"

class WebPortal {
public:
  WebPortal();
  void begin();
  void loop();

private:
  WebServer _server;
  String _sessionToken;
  uint8_t _sessionRole = 0; // 0 none, 1 viewer, 2 operador, 3 admin
  File _uploadFile;
  String _uploadPath;

  // Verdadeiro quando o cartao nao aceitou tudo durante o upload. Sem isto o
  // portal respondia "Upload concluido" para um arquivo gravado pela metade.
  bool _uploadFalhou = false;

  bool authenticated(uint8_t minRole = 1);
  bool requireAuth(uint8_t minRole = 1);
  String randomToken();
  void sendJson(int code, const String& json);
  void sendPlain(int code, const String& text);
  void registerRoutes();

  void handleRoot();
  void handleLoginPage();
  void handleLoginApi();
  void handleLogout();
  void handleStatus();
  void handleFiles();
  void handleMkdir();
  void handleDelete();
  void handleRename();
  void handleCopy();
  void handleMove();
  void handleDownload();
  void handleUpload();
  void handleUploadDone();
  void handleCommand();
  void handleSerialGet();
  void handleSerialSet();
  void handleNetworkGet();
  void handleNetworkSet();
  void handleSenhasSet();
  void handleLogs();
};

extern WebPortal webPortal;
