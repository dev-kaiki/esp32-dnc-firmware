#include "Logger.h"
#include <stdarg.h>

QueueHandle_t gLogQueue = nullptr;

void logEvent(const char* level, const char* text) {
  if (!gLogQueue) return;
  LogMessage msg;
  memset(&msg, 0, sizeof(msg));
  strncpy(msg.level, level ? level : "INFO", sizeof(msg.level) - 1);
  strncpy(msg.text, text ? text : "", sizeof(msg.text) - 1);
  msg.ms = millis();
  xQueueSend(gLogQueue, &msg, 0);
}

void logEventf(const char* level, const char* fmt, ...) {
  char buf[180];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  logEvent(level, buf);
}

String escapeJson(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if ((uint8_t)c < 32) out += ' ';
        else out += c;
    }
  }
  return out;
}
