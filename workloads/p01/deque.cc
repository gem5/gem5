#include "deque.hh"

template <typename T>
DequeCL<T>::DequeCL(uint64_t capacidade)
  : deque(),
    topo(0),
    base(0),
    mascara(capacidade - 1)
    {
      if (capacidade == 0 || (capacidade & mascara) != 0) {
        throw std::invalid_argument("A capacidade da deque deve ser potência de 2");
      }
      deque.resize(capacidade);
    }


template <typename T>
bool
DequeCL<T>::empilhar(const T &t) {  
  int64_t top = topo.load();
  if (top - base.load() > static_cast<int64_t>(mascara)) return false;
  
  deque[top & mascara] = t;
  
  topo.store(top + 1);

  return true;
}

template <typename T>
bool
DequeCL<T>::desempilhar(T &t) {
  // Decrementa do topo antes de ler a base
  int64_t top = topo.load() - 1;
  
  topo.store(top);

  int64_t bottom = base.load();

  // Verifica se está vazia e se estiver desfaz a reserva
  if (bottom > top) {
    topo.store(top + 1);
    return false;
  }
  
  t = deque[top & mascara];

  // Se topo == base, há apenas uma tarefa, que deverá ser disputada
  if (top == bottom) {
    bool ganhou = base.compare_exchange_strong(bottom, bottom + 1);
    topo.store(top + 1);
    return ganhou;
  }

  return true;
}

template <typename T>
bool
DequeCL<T>::roubar(T &t) {
  int64_t bottom = base.load();
  int64_t top = topo.load();
  
  if (bottom >= top) return false;

  T copia = deque[bottom & mascara];

  if (!base.compare_exchange_strong(bottom, bottom + 1)) {return false;}
  t = copia;

  return true;
}

template <typename T>
bool
DequeCL<T>::vazia() const { return topo.load() <= base.load(); }

template <typename T>
bool
DequeCL<T>::cheia() const { 
  return topo.load() - base.load() > static_cast<int64_t>(mascara);
}

#include "deque.tpp"