/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Kevin Lim
 */

#include <string>

#include "cpu/checker/cpu.hh"
#include "cpu/inst_seq.hh"
#include "cpu/ozone/dyn_inst.hh"
#include "cpu/ozone/ozone_impl.hh"
#include "mem/base_mem.hh"
#include "sim/builder.hh"
#include "sim/process.hh"
#include "sim/sim_object.hh"

/**
 * Specific non-templated derived class used for SimObject configuration.
 */
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
    Param<bool> warnOnlyOnLoadError;
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
    INIT_PARAM_DFLT(warnOnlyOnLoadError, "warn, but don't exit, if a load "
                    "result errors", false),
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
    params->warnOnlyOnLoadError = warnOnlyOnLoadError;
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
