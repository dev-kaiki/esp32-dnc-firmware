#include "StorageManager.h"
#include "Logger.h"
#include "../Config.h"

bool StorageManager::begin() {
  if (!_sdMutex) _sdMutex = xSemaphoreCreateMutex();
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  _ok = SD.begin(PIN_SD_CS, SPI, 25000000);
  if (_ok) ensureBaseDirs();
  return _ok;
}

bool StorageManager::isOk() const {
  return _ok;
}

SemaphoreHandle_t StorageManager::mutex() {
  return _sdMutex;
}

String StorageManager::sanitizePath(const String& input) {
  String p = input;
  p.trim();
  p.replace("\\", "/");
  while (p.indexOf("//") >= 0) p.replace("//", "/");
  p.replace("..", "");
  if (!p.startsWith("/")) p = "/" + p;
  if (p.length() > 1 && p.endsWith("/")) p.remove(p.length() - 1);
  return p;
}

bool StorageManager::ensureBaseDirs() {
  if (!_ok) return false;
  makeDir(DIR_PROGRAMS);
  makeDir(DIR_RECEIVED);
  makeDir(DIR_BACKUPS);
  makeDir(DIR_LOGS);
  makeDir(DIR_TMP);
  return true;
}

bool StorageManager::makeDir(const String& path) {
  if (!_ok) return false;
  String p = sanitizePath(path);
  bool ok;
  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  ok = SD.exists(p) || SD.mkdir(p);
  xSemaphoreGive(_sdMutex);
  return ok;
}

bool StorageManager::exists(const String& path) {
  if (!_ok) return false;
  String p = sanitizePath(path);
  bool ok;
  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  ok = SD.exists(p);
  xSemaphoreGive(_sdMutex);
  return ok;
}

uint64_t StorageManager::fileSize(const String& path) {
  if (!_ok) return 0;
  String p = sanitizePath(path);
  uint64_t size = 0;
  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  File f = SD.open(p, FILE_READ);
  if (f && !f.isDirectory()) size = f.size();
  if (f) f.close();
  xSemaphoreGive(_sdMutex);
  return size;
}

uint64_t StorageManager::totalBytes() {
  return _ok ? SD.totalBytes() : 0;
}

uint64_t StorageManager::usedBytes() {
  return _ok ? SD.usedBytes() : 0;
}

uint64_t StorageManager::freeBytes() {
  uint64_t t = totalBytes();
  uint64_t u = usedBytes();
  return t > u ? t - u : 0;
}

String StorageManager::listJson(const String& path) {
  String p = sanitizePath(path);
  String json = "{\"path\":\"" + escapeJson(p) + "\",\"items\":[";
  if (!_ok) return json + "]}";

  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  File root = SD.open(p);
  if (root && root.isDirectory()) {
    bool first = true;
    File file = root.openNextFile();
    while (file) {
      String name = String(file.name());
      int slash = name.lastIndexOf('/');
      if (slash >= 0) name = name.substring(slash + 1);
      if (!first) json += ",";
      first = false;
      json += "{\"name\":\"" + escapeJson(name) + "\",";
      json += "\"path\":\"" + escapeJson(String(file.path())) + "\",";
      json += "\"dir\":" + String(file.isDirectory() ? "true" : "false") + ",";
      json += "\"size\":" + String((uint64_t)file.size()) + "}";
      file.close();
      file = root.openNextFile();
    }
  }
  if (root) root.close();
  xSemaphoreGive(_sdMutex);
  json += "]}";
  return json;
}

bool StorageManager::removeRecursive(const String& path) {
  File f = SD.open(path);
  if (!f) return false;
  if (!f.isDirectory()) {
    f.close();
    return SD.remove(path);
  }
  File child = f.openNextFile();
  while (child) {
    String cp = String(child.path());
    child.close();
    removeRecursive(cp);
    child = f.openNextFile();
  }
  f.close();
  return SD.rmdir(path);
}

bool StorageManager::removePath(const String& path) {
  if (!_ok) return false;
  String p = sanitizePath(path);
  if (p == "/") return false;
  bool ok;
  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  ok = removeRecursive(p);
  xSemaphoreGive(_sdMutex);
  return ok;
}

bool StorageManager::renamePath(const String& from, const String& to) {
  if (!_ok) return false;
  String a = sanitizePath(from);
  String b = sanitizePath(to);
  bool ok;
  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  ok = SD.rename(a, b);
  xSemaphoreGive(_sdMutex);
  return ok;
}

bool StorageManager::copyFile(const String& from, const String& to) {
  if (!_ok) return false;
  String a = sanitizePath(from);
  String b = sanitizePath(to);
  bool ok = false;
  uint8_t buf[512];
  xSemaphoreTake(_sdMutex, portMAX_DELAY);
  File src = SD.open(a, FILE_READ);
  File dst = SD.open(b, FILE_WRITE);
  if (src && dst && !src.isDirectory()) {
    ok = true;
    while (src.available()) {
      size_t n = src.read(buf, sizeof(buf));
      if (dst.write(buf, n) != n) { ok = false; break; }
      delay(0);
    }
  }
  if (src) src.close();
  if (dst) dst.close();
  xSemaphoreGive(_sdMutex);
  return ok;
}

bool StorageManager::movePath(const String& from, const String& to) {
  return renamePath(from, to);
}
