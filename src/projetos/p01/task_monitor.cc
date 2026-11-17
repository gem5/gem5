#include "projetos/p01/task_monitor.hh"

#include "debug/TaskMonitor.hh"

#include <algorithm>

namespace gem5
{

TaskMonitor *TaskMonitor::_instance = nullptr;

TaskMonitor::TaskMonitor(const Params &p)
  : SimObject(p),
    listeners(p.cpus.size()),
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
  DPRINTF(TaskMonitor, "ctx=%d, tick=%llu, tipo=%llu, arg=%llu\n",
                        ctx, when, tipo, arg);

  if (ctx < 0 || ctx >= (ContextID)ctxs.size()) {
    stats.invalidEvents++;
    return;
  }

  ContextState &state = ctxs[ctx];

  // arg varia pra cada case
  switch (tipo)
  {
  case TASK_BEGIN:
    if (not state.inTask) {
      state.inTask = true;
      state.taskID = arg; 
      state.taskStart = when;
    }
    else stats.invalidEvents++; 
    break;
    
  case TASK_END:
    if (state.inTask) {
      Tick inTaskTime = when - state.taskStart;
      state.inTask = false;
      stats.tasksExecuted[ctx]++;
      stats.ticksInTask[ctx] += inTaskTime;
      stats.taskDuration.sample(inTaskTime);
    }
    else stats.invalidEvents++; 
    break;
    
  case STEAL_TRY:
    if (not state.inTask) {
      stats.stealAttempts[ctx]++;
    }
    else stats.invalidEvents++;
    break;
    
  case STEAL_OK:
    if (not state.inTask) {
      stats.stealSuccesses[ctx]++;
    }
    else stats.invalidEvents++;
    break;
  
  default:
    stats.invalidEvents++;
  }
}

void
TaskMonitor::startup() { 
  SimObject::startup();
  for (size_t i = 0; i < params().cpus.size(); i++) {
    BaseCPU *cpu = params().cpus[i];

    fatal_if(cpu->numContexts() != 1, 
            "TaskMonitor: a CPU %d deve ter exatamente 1 contexto", i);

    fatal_if(cpu->getContext(0)->contextId() != (ContextID) i,
            "TaskMonitor: a CPU %d tem o contexto %d", i,
             cpu->getContext(0)->contextId());
  }
}

void
TaskMonitor::instCtxCounter(unsigned ctx, uint64_t n) {
  if (ctxs[ctx].inTask) { stats.instsInTask[ctx] += n; }
  else { stats.instsOutside[ctx] += n; } 
}

void
TaskMonitor::regProbeListeners() {
  BaseCPU *cpu;
  for (size_t i = 0; i < params().cpus.size(); i++) {
    cpu = params().cpus[i];
    listeners[i] = cpu->getProbeManager()->connect<InstListener>(
      "RetiredInsts", this, i);
  }
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
  taskDuration.init(0, 100000000000ULL, 1000000000ULL);
};

void
TaskMonitor::MonitorStats::preDumpStats() {
  statistics::Group::preDumpStats();

  double maxTicks = 0.0;
  double somaTicks = 0.0;
  const size_t n = ticksInTask.size();
  for (size_t i = 0; i < n; i++) {
    const double ticks = ticksInTask[i].value();
    maxTicks = std::max(maxTicks, ticks);
    somaTicks += ticks;
  }
  imbalance = (somaTicks > 0.0) ? maxTicks / (somaTicks / n) : 0.0;
}
} // namespace
