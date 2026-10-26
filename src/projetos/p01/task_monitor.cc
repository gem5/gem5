#include "projetos/p01/task_monitor.hh"

#include "debug/TaskMonitor.hh"

namespace gem5
{

TaskMonitor *TaskMonitor::_instance = nullptr;

TaskMonitor::TaskMonitor(const Params &p)
    : SimObject(p),
      ctxs(p.cpus.size()),
    stats(this, p.cpus.size())
    {
      fatal_if(_instance, "Apenas um TaskMonitor é permitido no sistema");
      _instance = this;
    }

TaskMonitor::~TaskMonitor() {
    if (_instance == this) _instance = nullptr;
}

void
TaskMonitor::event(ContextID ctx, Tick when, uint64_t tipo, uint64_t arg) {
  DPRINTF(TaskMonitor, "ctx=%d, tick=%llu, tipo=%llu, arg=%llu",
                        ctx, when, tipo, arg);
}

void
TaskMonitor::startup() { return; }

void
TaskMonitor::regProbeListeners() {
  return;
}






Tick
TaskMonitor::mediaTicksInTask(std::vector<Tick> ticksInTask) {
  Tick sum = std::accumulate(ticksInTask.begin(), ticksInTask.end(), 0);

  return sum / ticksInTask.size();
}


TaskMonitor::MonitorStats::MonitorStats(TaskMonitor *parent, size_t num_cpus)
  : statistics::Group(parent),
    ADD_STAT(tasksExecuted, statistics::units::Count::get(),
              "Números de Tasks executadas por contexto"),
    ADD_STAT(stealAttempts, statistics::units::Count::get(),
              "Números de tentativas de roubos por contexto"),
    ADD_STAT(stealSuccesses, statistics::units::Count::get(),
              "Número de roubos sucedidos por contexto"),
    ADD_STAT(ticksInTask, statistics::units::Tick::get(),
              "Quantidades de ticks dentro de tasks por contexto"),
    ADD_STAT(instsInTask, statistics::units::Count::get(),
              "Quantidade de instruções executadas dentro de"
              " task por contexto"),
    ADD_STAT(instsOutside, statistics::units::Count::get(),
              "Quantidades de instruções executadas fora"
              " de task por contexto"),
    ADD_STAT(taskDuration, statistics::units::Tick::get(),
              "Distribuição da duração das tasks em ticks"),
    ADD_STAT(invalidEvents, statistics::units::Count::get(),
              "Quantidade total de eventos inválidos na"
              " máquina de estados (Deve ser 0)"),
    ADD_STAT(imbalance, statistics::units::Count::get(),
              "Desequilíbrio de trabalho dos contextos"),
    ADD_STAT(stealSuccessRate, statistics::units::\
      Rate<statistics::units::Count,
      statistics::units::Count>::get(),
      "Taxa de roubos bem sucedidos",
      statistics::sum(stealSuccesses) / statistics::sum(stealAttempts))

{
  tasksExecuted.init(num_cpus);
  stealAttempts.init(num_cpus);
  stealSuccesses.init(num_cpus);
  ticksInTask.init(num_cpus);
  instsInTask.init(num_cpus);
  instsOutside.init(num_cpus);
  // (ticks minimo, ticks máximo, tamanho das faixas do histograma
  taskDuration.init(0, 10000, 1000);
};

void
TaskMonitor::MonitorStats::preDumpStats() {
  return;
}
} // namespace
