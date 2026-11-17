#include "tarefas.hh"

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
    std::cout << "Política do escalonador de tarefas não implementada\n" << "Roubo" << std::endl;
    id_trabalhador = 0;
    /* code */
    break;
  }
  trabalhadores[id_trabalhador].inserirTarefa(t);
}

void
Escalonador::trabalhar(Trabalhador &w) {
  switch (politica)
  {
  case Politica::Estatica:
    laco_estatico(w);
    break;

  case Politica::Roubo:
    laco_estatico(w);
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
Escalonador::get_semente() {
  return semente;
}
void
Escalonador::tarefa_concluida() {
  tarefas_pendentes--;
}

/* -- Trabalhador -- */

Trabalhador::Trabalhador(unsigned id, Escalonador *esc)
  : id(id),
    esc(esc),
    estado(0),
    executadas(0),
    tarefas()
    {
      estado = esc->get_semente() ^ (0x9E3779B97F4A7C15ull * (id + 1));
      if (estado == 0) estado = 12345678ULL; // Seed padrão
    }

void
Trabalhador::gerar(FuncTarefa f, void *dados) { 
  inserirTarefa(esc->criar_tarefa(f, dados));
}

unsigned
Trabalhador::indice() const { return id; } 

void 
Trabalhador::inserirTarefa(const Tarefa &t) {
  tarefas.push_back(t);
}

bool
Trabalhador::prox_tarefa(Tarefa &saida) {
  if (tarefas.empty()) return false;

  saida = tarefas.back();
  tarefas.pop_back();
  return true;
}

void
Trabalhador::tarefa_concluida() {
  executadas++;
  esc->tarefa_concluida();
}

uint64_t
Trabalhador::tarefas_executadas() const { return executadas; }

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