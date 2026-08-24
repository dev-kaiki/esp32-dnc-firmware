#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class CircularBuffer {
public:
  CircularBuffer();
  ~CircularBuffer();

  bool begin(size_t size);
  bool push(uint8_t value);
  bool pushBlock(const uint8_t* data, size_t len, size_t* pushed = nullptr);
  int pop();
  size_t popBlock(uint8_t* out, size_t maxLen);
  size_t available();
  size_t freeSpace();
  size_t capacity() const;
  void clear();

private:
  uint8_t* _data;
  size_t _size;
  size_t _head;
  size_t _tail;
  size_t _count;
  SemaphoreHandle_t _mutex;
};
