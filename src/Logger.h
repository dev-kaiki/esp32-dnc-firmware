#pragma once
#include <Arduino.h>
#include "Types.h"

extern QueueHandle_t gLogQueue;

void logEvent(const char* level, const char* text);
void logEventf(const char* level, const char* fmt, ...);
String escapeJson(const String& in);
