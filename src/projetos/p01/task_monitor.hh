#ifndef __PROJETOS_P01_TASK_MONITOR_HH__
#define __PROJETOS_P01_TASK_MONITOR_HH__

#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "params/TaskMonitor.hh"
#include "sim/sim_object.hh"
#include "gem5/p01_task_events.h"
#include "sim/probe/probe.hh"
#include "cpu/base.hh"

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

    void instCtxCounter(unsigned ctx, uint64_t n);

    void regProbeListeners() override;

  private:
    // Listener das instruções executadas
    class InstListener : public ProbeListenerArgBase<uint64_t> {
      public:
        InstListener(const std::string &name, TaskMonitor *m, unsigned ctx)
          : ProbeListenerArgBase(name), monitor(m), ctx(ctx) {}
          
        // n é a quantidade de instruções rodadas em um contexto ou fora dele
        void notify(const uint64_t &n) override {
          monitor->instCtxCounter(ctx, n);
        }

      private:
        TaskMonitor *monitor;
        unsigned ctx;
    };

    static TaskMonitor *_instance;
    std::vector<ProbeListenerPtr<InstListener>> listeners; // InstListener declarado abaixo

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
