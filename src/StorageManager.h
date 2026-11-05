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

  // Transforma o nome vindo de um upload em algo gravavel no cartao.
  //
  // POR QUE PRECISA EXISTIR: o analisador de formulario do WebServer do ESP32
  // supoe que a linha `Content-Disposition` TERMINA na aspa que fecha o nome do
  // arquivo (Parsing.cpp: `substring(nameStart + 2, argName.length() - 1)`).
  // Havendo qualquer parametro depois -- e todo navegador acrescenta
  // `filename*=utf-8''...` assim que o nome tem acento -- o nome chega
  // contaminado com o resto do cabecalho, e o arquivo vai parar no cartao com um
  // nome que ninguem consegue abrir depois. Observado em bancada.
  //
  // Nao corrigimos a biblioteca porque a correcao sumiria na proxima
  // atualizacao do core. Aqui o estrago e desfeito: o que sobra e
  // `nome.nc"; filename*=...`, e cortar na primeira aspa devolve o nome exato.
  // ESTATICO DE PROPOSITO.
  //
  // Quem GRAVA o arquivo (o servidor de protocolo) e quem PROCURA o arquivo
  // (o comando de envio) precisam usar exatamente a mesma regra. Enquanto cada
  // lado tinha a sua, um nome longo era gravado inteiro e procurado truncado
  // em 64 -- e o operador via so "Falha", sem nenhuma pista de que o problema
  // era o nome do proprio programa.
  static String sanitizeFileName(const String& bruto);

  // GRAVA TUDO, INSISTINDO NO QUE O CARTAO NAO ACEITOU DE PRIMEIRA.
  //
  // Escrita curta e comportamento NORMAL de cartao -- um instante de
  // apagamento de bloco, atualizacao de FAT ou disputa pelo barramento faz
  // write() aceitar menos do que foi pedido. Tratar isso como erro fatal era a
  // causa do "programa nao aceito pelo SD" que aparecia de forma aleatoria: o
  // mesmo arquivo ora passava ora nao, porque o que decidia era o instante da
  // gravacao, nao o arquivo.
  //
  // `mutex` pode ser nulo. Quando vem, e tomado e solto A CADA TENTATIVA, para
  // que a tarefa que le o programa para a maquina nao fique esperando o cartao
  // durante as pausas.
  //
  // Devolve quantos bytes foram gravados; menor que `n` significa cartao que
  // parou de aceitar de verdade.
  static size_t escreverTudo(File& destino, const uint8_t* dados, size_t n,
                             SemaphoreHandle_t mutex = nullptr);

  SemaphoreHandle_t mutex();

private:
  bool _ok = false;
  SemaphoreHandle_t _sdMutex = nullptr;
  bool removeRecursive(const String& path);
};
