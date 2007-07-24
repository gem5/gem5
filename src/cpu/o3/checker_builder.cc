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

#include "cpu/checker/cpu_impl.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/alpha/dyn_inst.hh"
#include "cpu/o3/alpha/impl.hh"
#include "params/O3Checker.hh"
#include "sim/process.hh"
#include "sim/sim_object.hh"

class MemObject;

template
class Checker<RefCountingPtr<AlphaDynInst<AlphaSimpleImpl> > >;

/**
 * Specific non-templated derived class used for SimObject configuration.
 */
class O3Checker : public Checker<RefCountingPtr<AlphaDynInst<AlphaSimpleImpl> > >
{
  public:
    O3Checker(Params *p)
        : Checker<RefCountingPtr<AlphaDynInst<AlphaSimpleImpl> > >(p)
    { }
};

////////////////////////////////////////////////////////////////////////
//
//  CheckerCPU Simulation Object
//
O3Checker *
O3CheckerParams::create()
{
    O3Checker::Params *params = new O3Checker::Params();
    params->name = name;
    params->numberOfThreads = 1;
    params->max_insts_any_thread = 0;
    params->max_insts_all_threads = 0;
    params->max_loads_any_thread = 0;
    params->max_loads_all_threads = 0;
    params->exitOnError = exitOnError;
    params->updateOnError = updateOnError;
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
    Tick temp2 = progress_interval;
    params->progress_interval = 0;
    temp2++;

#if FULL_SYSTEM
    params->itb = itb;
    params->dtb = dtb;
    params->system = system;
    params->cpu_id = cpu_id;
    params->profile = profile;
#else
    params->process = workload;
#endif

    O3Checker *cpu = new O3Checker(params);
    return cpu;
}
