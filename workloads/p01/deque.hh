#pragma once
#include <atomic>
#include <cstdint>
#include <stdexcept>
#include <iostream>
#include <vector>

template <typename T>
class DequeCL {
  public:
    explicit DequeCL(uint64_t capacidade); // potencia de 2

    /* Dono: devolve false se a fila estiver cheia */
    bool empilhar(const T &t);

    /* 
    Dono: devolve false se a fila estiver vazia ou se perdeu a disputa pelo 
    último elemento
    */
    bool desempilhar(T &t);

    /* Ladrão: devolve false se a fila parece vazia ou se perdeu a disputa */
    bool roubar(T &t);

    bool vazia() const;

    bool cheia() const;

  private:
    // todo: decidir se topo e base ficam na mesma linha de cache ou em linhas 
    //       separadas, e medir o efeito
    std::vector<T> deque;
    alignas(64) std::atomic<int64_t> topo;
    alignas(64) std::atomic<int64_t> base;
    uint64_t mascara;
    // todo: vetor circular de tarefas e máscara de índice 
};
