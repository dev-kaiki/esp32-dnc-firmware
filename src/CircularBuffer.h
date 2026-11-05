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

  // ESPIAR E DESCARTAR, em vez de retirar de uma vez.
  //
  // POR QUE ISTO EXISTE: quem alimenta a CNC precisa saber QUANTOS bytes a UART
  // realmente aceitou antes de dar o dado por consumido. Com `popBlock` os
  // bytes saem da fila no momento da leitura -- se a escrita aceitasse menos, o
  // que sobrou estaria perdido, e o resultado seria um programa chegando
  // truncado na maquina, sem erro em lugar nenhum. Foi exatamente essa classe
  // de defeito ("o envio para no meio do programa") que custou caro antes.
  //
  // Com espiar/descartar, o dado so sai da fila depois de confirmado no fio.
  size_t peekBlock(uint8_t* out, size_t maxLen);
  size_t discard(size_t n);

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
