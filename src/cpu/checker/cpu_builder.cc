
#include <string>

#include "cpu/checker/cpu.hh"
#include "cpu/inst_seq.hh"
#include "cpu/ozone/dyn_inst.hh"
#include "cpu/ozone/ozone_impl.hh"
#include "mem/base_mem.hh"
#include "sim/builder.hh"
#include "sim/process.hh"
#include "sim/sim_object.hh"

class OzoneChecker : public Checker<RefCountingPtr<OzoneDynInst<OzoneImpl> > >
{
  public:
    OzoneChecker(Params *p)
        : Checker<RefCountingPtr<OzoneDynInst<OzoneImpl> > >(p)
    { }
};

////////////////////////////////////////////////////////////////////////
//
//  CheckerCPU Simulation Object
//
BEGIN_DECLARE_SIM_OBJECT_PARAMS(OzoneChecker)

    Param<Counter> max_insts_any_thread;
    Param<Counter> max_insts_all_threads;
    Param<Counter> max_loads_any_thread;
    Param<Counter> max_loads_all_threads;

#if FULL_SYSTEM
    SimObjectParam<AlphaITB *> itb;
    SimObjectParam<AlphaDTB *> dtb;
    SimObjectParam<FunctionalMemory *> mem;
    SimObjectParam<System *> system;
    Param<int> cpu_id;
    Param<Tick> profile;
#else
    SimObjectParam<Process *> workload;
#endif // FULL_SYSTEM
    Param<int> clock;
    SimObjectParam<BaseMem *> icache;
    SimObjectParam<BaseMem *> dcache;

    Param<bool> defer_registration;
    Param<bool> exitOnError;
    Param<bool> function_trace;
    Param<Tick> function_trace_start;

END_DECLARE_SIM_OBJECT_PARAMS(OzoneChecker)

BEGIN_INIT_SIM_OBJECT_PARAMS(OzoneChecker)

    INIT_PARAM(max_insts_any_thread,
               "terminate when any thread reaches this inst count"),
    INIT_PARAM(max_insts_all_threads,
               "terminate when all threads have reached this inst count"),
    INIT_PARAM(max_loads_any_thread,
               "terminate when any thread reaches this load count"),
    INIT_PARAM(max_loads_all_threads,
               "terminate when all threads have reached this load count"),

#if FULL_SYSTEM
    INIT_PARAM(itb, "Instruction TLB"),
    INIT_PARAM(dtb, "Data TLB"),
    INIT_PARAM(mem, "memory"),
    INIT_PARAM(system, "system object"),
    INIT_PARAM(cpu_id, "processor ID"),
    INIT_PARAM(profile, ""),
#else
    INIT_PARAM(workload, "processes to run"),
#endif // FULL_SYSTEM

    INIT_PARAM(clock, "clock speed"),
    INIT_PARAM(icache, "L1 instruction cache object"),
    INIT_PARAM(dcache, "L1 data cache object"),

    INIT_PARAM(defer_registration, "defer system registration (for sampling)"),
    INIT_PARAM(exitOnError, "exit on error"),
    INIT_PARAM(function_trace, "Enable function trace"),
    INIT_PARAM(function_trace_start, "Cycle to start function trace")

END_INIT_SIM_OBJECT_PARAMS(OzoneChecker)


CREATE_SIM_OBJECT(OzoneChecker)
{
    OzoneChecker::Params *params = new OzoneChecker::Params();
    params->name = getInstanceName();
    params->numberOfThreads = 1;
    params->max_insts_any_thread = 0;
    params->max_insts_all_threads = 0;
    params->max_loads_any_thread = 0;
    params->max_loads_all_threads = 0;
    params->exitOnError = exitOnError;
    params->deferRegistration = defer_registration;
    params->functionTrace = function_trace;
    params->functionTraceStart = function_trace_start;
    params->clock = clock;
    // Hack to touch all parameters.  Consider not deriving Checker
    // from BaseCPU..it's not really a CPU in the end.
    Counter temp;
    temp = max_insts_any_thread;
    temp = max_insts_all_threads;
    temp = max_loads_any_thread;
    temp = max_loads_all_threads;
    BaseMem *cache = icache;
    cache = dcache;

#if FULL_SYSTEM
    params->itb = itb;
    params->dtb = dtb;
    params->mem = mem;
    params->system = system;
    params->cpu_id = cpu_id;
    params->profile = profile;
#else
    params->process = workload;
#endif

    OzoneChecker *cpu = new OzoneChecker(params);
    return cpu;
}

REGISTER_SIM_OBJECT("OzoneChecker", OzoneChecker)
