#include "tarefas.hh"
#include "deque.hh"

/* -- Escalonador -- */

Escalonador::Escalonador(unsigned num_trabalhadores, Politica p, uint64_t semente)
  : trabalhadores(),
    tarefas_pendentes(0),
    qtd_identificadores(0),
    start(false),
    politica(p),
    semente(semente)
    {
      if (num_trabalhadores == 0) {
        throw std::invalid_argument("O escalonador precisa de pelo menos um trabalhador");
      }

      trabalhadores.reserve(num_trabalhadores);
      for (unsigned i = 0; i < num_trabalhadores; i++) {
        trabalhadores.emplace_back(i, this);
      }
    }
  
Tarefa
Escalonador::criar_tarefa(FuncTarefa f, void *dados) {
  tarefas_pendentes++;
  return Tarefa{f, dados, qtd_identificadores++};
}

void
Escalonador::submeter(FuncTarefa f, void *dados) {
  Tarefa t = criar_tarefa(f, dados);
  unsigned id_trabalhador = 0;

  switch (politica)
  {
  case Politica::Estatica:
    id_trabalhador = t.id % trabalhadores.size();
    break;
  
  case Politica::Roubo:
    /* Todas as tarefas vão para o trabalhador 0, e os outros trabalhadores 
    roubam dele */
    id_trabalhador = 0;
    break;
  }

  if (!trabalhadores[id_trabalhador].inserirTarefa(t)) {
    throw std::runtime_error("Deque cheia: aumente CAPACIDADE_DEQUE");
  }
}

void
Escalonador::trabalhar(Trabalhador &w) {
  switch (politica)
  {
  case Politica::Estatica:
    laco_estatico(w);
    break;

  case Politica::Roubo:
    laco_roubo(w);
    break;
  }
}

uint64_t
Escalonador::executar() {
  unsigned num_trabalhadores = trabalhadores.size();
  std::vector<std::thread> threads;
  threads.reserve(num_trabalhadores-1);
  start.store(false);

  // Cria threads de 1 até n-1 em ordem, com o trabalhador principal sendo o 0
  for (unsigned i = 1; i < num_trabalhadores; i++) {
    threads.emplace_back([this, i] {
      // Todas as threads só podem começar quando os stats resetarem
      while (!start.load(std::memory_order_acquire)) {}
      trabalhar(trabalhadores[i]);
    });
  }  

  #ifdef USE_M5OPS
  m5_reset_stats(0, 0);
  #endif
  
  start.store(true, std::memory_order_release);

  trabalhar(trabalhadores[0]);

  for (std::thread &thread : threads) { thread.join(); }

  #ifdef USE_M5OPS
  m5_dump_stats(0, 0);
  #endif

  uint64_t total = 0;
  for (const Trabalhador &w : trabalhadores) total += w.tarefas_executadas();
  return total;
}

uint64_t
Escalonador::get_semente() { return semente; }

void
Escalonador::tarefa_concluida() { tarefas_pendentes--; }

unsigned
Escalonador::num_trabalhadores() const { return trabalhadores.size(); }

Trabalhador &
Escalonador::trabalhador(unsigned i) { return trabalhadores[i]; }

bool
Escalonador::ha_pendentes() const { return tarefas_pendentes > 0; }


/* -- Trabalhador -- */

Trabalhador::Trabalhador(unsigned id, Escalonador *esc)
  : id(id),
    esc(esc),
    estado(0),
    executadas(0),
    tarefas(std::make_unique<DequeCL<Tarefa>>(CAPACIDADE_DEQUE))
    {
      estado = esc->get_semente() ^ (0x9E3779B97F4A7C15ull * (id + 1));
      if (estado == 0) estado = 12345678ULL; // Seed padrão
    }

void
Trabalhador::gerar(FuncTarefa f, void *dados) { 
  Tarefa t = esc->criar_tarefa(f, dados);
  if (!inserirTarefa(t)) {
    t.func(t.dados, *this);
    tarefa_concluida();
  }
}

unsigned
Trabalhador::indice() const { return id; } 

bool 
Trabalhador::inserirTarefa(const Tarefa &t) {
  return tarefas->empilhar(t);
}

bool
Trabalhador::prox_tarefa(Tarefa &saida) {
  return tarefas->desempilhar(saida);
}

bool
Trabalhador::tentar_roubar(Tarefa &saida) {
  unsigned n = esc->num_trabalhadores();
  if (n <= 1) return false;

  uint64_t vitima_id = rand() % (n-1);

  if (vitima_id >= id) vitima_id++; 

  P01_EVENT(STEAL_TRY, vitima_id);

  if (!esc->trabalhador(vitima_id).tarefas->roubar(saida)) return false;

  P01_EVENT(STEAL_OK, vitima_id);

  return true;
}

void
Trabalhador::tarefa_concluida() {
  executadas++;
  esc->tarefa_concluida();
}

uint64_t
Trabalhador::tarefas_executadas() const { return executadas; }

bool
Trabalhador::ha_pendentes() const { return esc->ha_pendentes(); }

uint64_t
Trabalhador::rand() {
  // Algoritmo Xorshift64

  estado ^= estado << 13;
  estado ^= estado >> 7;
  estado ^= estado >> 17;
  return estado; 
}

void
laco_estatico(Trabalhador &t)
{
  Tarefa tarefa;
  while (t.prox_tarefa(tarefa)) {
    P01_EVENT(TASK_BEGIN, tarefa.id);
    tarefa.func(tarefa.dados, t);
    P01_EVENT(TASK_END, tarefa.id);
    t.tarefa_concluida();
  }
}

void
laco_roubo(Trabalhador &t) {
  Tarefa tarefa;
  while (t.ha_pendentes()) {
    if (t.prox_tarefa(tarefa) || t.tentar_roubar(tarefa)) {
      P01_EVENT(TASK_BEGIN, tarefa.id);
      tarefa.func(tarefa.dados, t);
      P01_EVENT(TASK_END, tarefa.id);
      t.tarefa_concluida();
    }
    #ifndef USE_M5OPS
      else std::this_thread::yield();   // só nativo; no gem5 é espera ativa
    #endif
  }
}