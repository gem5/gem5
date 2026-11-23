#pragma once
#include <atomic>
#include <vector>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include "deque.hh"

#ifdef USE_M5OPS
#include <gem5/m5ops.h>
#include <gem5/p01_task_events.h>
#define P01_EVENT(t, a) m5_task_event((t), (a))
#else
#define P01_EVENT(t, a) ((void)0)
#endif

constexpr uint64_t CAPACIDADE_DEQUE = 1 << 14;

enum class Politica { Estatica, Roubo };

class Trabalhador;
using FuncTarefa = void (*)(void *dados, Trabalhador &t);

struct Tarefa
{
    FuncTarefa func;
    void *dados;
    uint64_t id;
};

class Escalonador 
{
  public:
    Escalonador(unsigned num_trabalhadores, Politica p, uint64_t semente);
    ~Escalonador() = default;
    
    /* Submete uma tarefa inicial, antes de executar(). */
    void submeter(FuncTarefa f, void *dados);

    /* 
    Cria as threads, executa até não haver tarefas pendentes e devolve o número 
    total de tarefas executadas. 
    */
    uint64_t executar();

    uint64_t get_semente();

    /*
    Cria uma tarefa pendente com id único. 
    - Pode ser chamada por várias threads ao mesmo tempo
    */
    Tarefa criar_tarefa(FuncTarefa f, void *dados);

    /* Decrementa a quantidade de tarefas pendentes em 1 */
    void tarefa_concluida();

    // Funções usadas pelo ladrão

    unsigned num_trabalhadores() const;

    Trabalhador &trabalhador(unsigned i);

    bool ha_pendentes() const;

  private:
    /* Executa o laço do trabalhador `w` de acordo com sua política */
    void trabalhar(Trabalhador &w);

    std::vector<Trabalhador> trabalhadores;
    std::atomic<uint64_t> tarefas_pendentes;
    std::atomic<uint64_t> qtd_identificadores;
    std::atomic<bool> start;  // indica quando as threads podem começar as tarefas
    Politica politica;
    uint64_t semente;

};

class Trabalhador 
{
  public:
    Trabalhador(unsigned id, Escalonador *esc);

    /* Cria uma subtarefa a partir de dentro de uma tarefa. */
    void gerar(FuncTarefa f, void *dados);
    unsigned indice() const;

    /* Insere `t` no topo da deque `tarefas`, devolve false se estiver cheia */
    bool inserirTarefa(const Tarefa &t);
    
    /* 
    Tira a próxima tarefa da fila de tarefas e a copia em `saida`
     - Se a fila estiver vazia retorna `false`
    */
    bool prox_tarefa(Tarefa &saida);

    /* Ladrão sorteia uma vítima e tenta roubar uma tarefa dela */
    bool tentar_roubar(Tarefa &saida);
    
    /* Conta a tarefa como concluída e decrementa as pendentes do escalonador */
    void tarefa_concluida();

    /* Verifica quantas tarefas foram executadas */
    uint64_t tarefas_executadas() const;

    /* Verifica quantas tarefas estão pendentes */
    bool ha_pendentes() const;
  private:
    unsigned id;
    Escalonador *esc;
    uint64_t estado; // Estado para geração do número aleatório
    uint64_t executadas;
    std::unique_ptr<DequeCL<Tarefa>> tarefas;

    /* Gera um número aleatório baseado na semente do escalonador*/
    uint64_t rand();
};

void laco_estatico(Trabalhador &t);
void laco_roubo(Trabalhador &t);