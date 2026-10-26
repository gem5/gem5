#ifndef __PROJETOS_P01_TASK_MONITOR_HH__
#define __PROJETOS_P01_TASK_MONITOR_HH__

#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "params/TaskMonitor.hh"
#include "sim/sim_object.hh"
#include "gem5/p01_task_events.h"

namespace gem5
{

class TaskMonitor : public SimObject
{

  public:
    PARAMS(TaskMonitor);
    TaskMonitor(const Params &p);
    ~TaskMonitor() override;

    static TaskMonitor *instance() { return _instance; }

    void event(ContextID ctx, Tick when, uint64_t tipo, uint64_t arg);

    void startup() override;

    void regProbeListeners() override;

  private:
    static TaskMonitor *_instance;

    // Contexto das Threads
    struct ContextState
    {
      bool inTask;
      Tick taskStart;
      uint64_t taskID;
    };

    std::vector<ContextState> ctxs;

    // Usado em cálculo de estatística
    Tick mediaTicksInTask(std::vector<Tick> ticksInTask);

    struct MonitorStats : public statistics::Group
    {
      MonitorStats(TaskMonitor *parent, size_t num_cpus);
      void preDumpStats() override;

      statistics::Vector tasksExecuted;
      statistics::Vector stealAttempts;
      statistics::Vector stealSuccesses;
      statistics::Vector ticksInTask;
      statistics::Vector instsInTask;
      statistics::Vector instsOutside;
      statistics::Distribution taskDuration;
      statistics::Scalar invalidEvents;
      statistics::Scalar imbalance;
      statistics::Formula stealSuccessRate;
    } stats;

};
} // namespace
#endif
