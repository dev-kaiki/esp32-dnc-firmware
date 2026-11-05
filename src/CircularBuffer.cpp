#include "CircularBuffer.h"

CircularBuffer::CircularBuffer()
  : _data(nullptr), _size(0), _head(0), _tail(0), _count(0), _mutex(nullptr) {}

CircularBuffer::~CircularBuffer() {
  if (_data) free(_data);
  if (_mutex) vSemaphoreDelete(_mutex);
}

bool CircularBuffer::begin(size_t size) {
  _data = (uint8_t*)malloc(size);
  if (!_data) return false;
  _size = size;
  _head = _tail = _count = 0;
  _mutex = xSemaphoreCreateMutex();
  return _mutex != nullptr;
}

bool CircularBuffer::push(uint8_t value) {
  bool ok = false;
  if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
    if (_count < _size) {
      _data[_head] = value;
      _head = (_head + 1) % _size;
      _count++;
      ok = true;
    }
    xSemaphoreGive(_mutex);
  }
  return ok;
}

bool CircularBuffer::pushBlock(const uint8_t* data, size_t len, size_t* pushed) {
  size_t localPushed = 0;
  if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
    while (localPushed < len && _count < _size) {
      _data[_head] = data[localPushed++];
      _head = (_head + 1) % _size;
      _count++;
    }
    xSemaphoreGive(_mutex);
  }
  if (pushed) *pushed = localPushed;
  return localPushed == len;
}

int CircularBuffer::pop() {
  int value = -1;
  if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
    if (_count > 0) {
      value = _data[_tail];
      _tail = (_tail + 1) % _size;
      _count--;
    }
    xSemaphoreGive(_mutex);
  }
  return value;
}

size_t CircularBuffer::peekBlock(uint8_t* out, size_t maxLen) {
  size_t n = 0;
  if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
    size_t pos = _tail;
    while (n < maxLen && n < _count) {
      out[n++] = _data[pos];
      pos = (pos + 1) % _size;
    }
    xSemaphoreGive(_mutex);
  }
  return n;
}

size_t CircularBuffer::discard(size_t n) {
  size_t descartados = 0;
  if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
    while (descartados < n && _count > 0) {
      _tail = (_tail + 1) % _size;
      _count--;
      ++descartados;
    }
    xSemaphoreGive(_mutex);
  }
  return descartados;
}

size_t CircularBuffer::popBlock(uint8_t* out, size_t maxLen) {
  size_t n = 0;
  if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
    while (n < maxLen && _count > 0) {
      out[n++] = _data[_tail];
      _tail = (_tail + 1) % _size;
      _count--;
    }
    xSemaphoreGive(_mutex);
  }
  return n;
}

size_t CircularBuffer::available() {
  size_t v = 0;
  if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
    v = _count;
    xSemaphoreGive(_mutex);
  }
  return v;
}

size_t CircularBuffer::freeSpace() {
  size_t v = 0;
  if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
    v = _size - _count;
    xSemaphoreGive(_mutex);
  }
  return v;
}

size_t CircularBuffer::capacity() const {
  return _size;
}

void CircularBuffer::clear() {
  if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
    _head = _tail = _count = 0;
    xSemaphoreGive(_mutex);
  }
}
