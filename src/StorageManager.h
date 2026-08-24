#pragma once
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

class StorageManager {
public:
  bool begin();
  bool isOk() const;
  bool ensureBaseDirs();
  String listJson(const String& path);
  bool makeDir(const String& path);
  bool removePath(const String& path);
  bool renamePath(const String& from, const String& to);
  bool copyFile(const String& from, const String& to);
  bool movePath(const String& from, const String& to);
  bool exists(const String& path);
  uint64_t fileSize(const String& path);
  uint64_t totalBytes();
  uint64_t usedBytes();
  uint64_t freeBytes();
  String sanitizePath(const String& path);
  SemaphoreHandle_t mutex();

private:
  bool _ok = false;
  SemaphoreHandle_t _sdMutex = nullptr;
  bool removeRecursive(const String& path);
};
