/*
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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
 */

#include <string>
#include <sstream>
#include <iostream>

#include "cpu/base_cpu.hh"
#include "base/cprintf.hh"
#include "cpu/exec_context.hh"
#include "base/misc.hh"
#include "sim/param.hh"
#include "sim/sim_events.hh"

using namespace std;

vector<BaseCPU *> BaseCPU::cpuList;

// This variable reflects the max number of threads in any CPU.  Be
// careful to only use it once all the CPUs that you care about have
// been initialized
int maxThreadsPerCPU = 1;

#ifdef FULL_SYSTEM
BaseCPU::BaseCPU(const string &_name, int _number_of_threads,
                 Counter max_insts_any_thread,
                 Counter max_insts_all_threads,
                 Counter max_loads_any_thread,
                 Counter max_loads_all_threads,
                 System *_system, Tick freq)
    : SimObject(_name), frequency(freq),
      number_of_threads(_number_of_threads), system(_system)
#else
BaseCPU::BaseCPU(const string &_name, int _number_of_threads,
                 Counter max_insts_any_thread,
                 Counter max_insts_all_threads,
                 Counter max_loads_any_thread,
                 Counter max_loads_all_threads)
    : SimObject(_name), number_of_threads(_number_of_threads)
#endif
{
    // add self to global list of CPUs
    cpuList.push_back(this);

    if (number_of_threads > maxThreadsPerCPU)
        maxThreadsPerCPU = number_of_threads;

    // allocate per-thread instruction-based event queues
    comInstEventQueue = new (EventQueue *)[number_of_threads];
    for (int i = 0; i < number_of_threads; ++i)
        comInstEventQueue[i] = new EventQueue("instruction-based event queue");

    //
    // set up instruction-count-based termination events, if any
    //
    if (max_insts_any_thread != 0)
        for (int i = 0; i < number_of_threads; ++i)
            new SimExitEvent(comInstEventQueue[i], max_insts_any_thread,
                "a thread reached the max instruction count");

    if (max_insts_all_threads != 0) {
        // allocate & initialize shared downcounter: each event will
        // decrement this when triggered; simulation will terminate
        // when counter reaches 0
        int *counter = new int;
        *counter = number_of_threads;
        for (int i = 0; i < number_of_threads; ++i)
            new CountedExitEvent(comInstEventQueue[i],
                "all threads reached the max instruction count",
                max_insts_all_threads, *counter);
    }

    // allocate per-thread load-based event queues
    comLoadEventQueue = new (EventQueue *)[number_of_threads];
    for (int i = 0; i < number_of_threads; ++i)
        comLoadEventQueue[i] = new EventQueue("load-based event queue");

    //
    // set up instruction-count-based termination events, if any
    //
    if (max_loads_any_thread != 0)
        for (int i = 0; i < number_of_threads; ++i)
            new SimExitEvent(comLoadEventQueue[i], max_loads_any_thread,
                "a thread reached the max load count");

    if (max_loads_all_threads != 0) {
        // allocate & initialize shared downcounter: each event will
        // decrement this when triggered; simulation will terminate
        // when counter reaches 0
        int *counter = new int;
        *counter = number_of_threads;
        for (int i = 0; i < number_of_threads; ++i)
            new CountedExitEvent(comLoadEventQueue[i],
                "all threads reached the max load count",
                max_loads_all_threads, *counter);
    }

#ifdef FULL_SYSTEM
    memset(interrupts, 0, sizeof(interrupts));
    intstatus = 0;
#endif
}


void
BaseCPU::regStats()
{
    using namespace Stats;

    numCycles
        .name(name() + ".numCycles")
        .desc("number of cpu cycles simulated")
        ;

    int size = execContexts.size();
    if (size > 1) {
        for (int i = 0; i < size; ++i) {
            stringstream namestr;
            ccprintf(namestr, "%s.ctx%d", name(), i);
            execContexts[i]->regStats(namestr.str());
        }
    } else if (size == 1)
        execContexts[0]->regStats(name());
}


void
BaseCPU::registerExecContexts()
{
    for (int i = 0; i < execContexts.size(); ++i) {
        ExecContext *xc = execContexts[i];
        int cpu_id;

#ifdef FULL_SYSTEM
        cpu_id = system->registerExecContext(xc);
#else
        cpu_id = xc->process->registerExecContext(xc);
#endif

        xc->cpu_id = cpu_id;
    }
}


void
BaseCPU::switchOut()
{
    // default: do nothing
}

void
BaseCPU::takeOverFrom(BaseCPU *oldCPU)
{
    assert(execContexts.size() == oldCPU->execContexts.size());

    for (int i = 0; i < execContexts.size(); ++i) {
        ExecContext *newXC = execContexts[i];
        ExecContext *oldXC = oldCPU->execContexts[i];

        newXC->takeOverFrom(oldXC);
        assert(newXC->cpu_id == oldXC->cpu_id);
#ifdef FULL_SYSTEM
        system->replaceExecContext(newXC->cpu_id, newXC);
#else
        assert(newXC->process == oldXC->process);
        newXC->process->replaceExecContext(newXC->cpu_id, newXC);
#endif
    }

#ifdef FULL_SYSTEM
    for (int i = 0; i < NumInterruptLevels; ++i)
        interrupts[i] = oldCPU->interrupts[i];
    intstatus = oldCPU->intstatus;
#endif
}


#ifdef FULL_SYSTEM
void
BaseCPU::post_interrupt(int int_num, int index)
{
    DPRINTF(Interrupt, "Interrupt %d:%d posted\n", int_num, index);

    if (int_num < 0 || int_num >= NumInterruptLevels)
        panic("int_num out of bounds\n");

    if (index < 0 || index >= sizeof(uint64_t) * 8)
        panic("int_num out of bounds\n");

    AlphaISA::check_interrupts = 1;
    interrupts[int_num] |= 1 << index;
    intstatus |= (ULL(1) << int_num);
}

void
BaseCPU::clear_interrupt(int int_num, int index)
{
    DPRINTF(Interrupt, "Interrupt %d:%d cleared\n", int_num, index);

    if (int_num < 0 || int_num >= NumInterruptLevels)
        panic("int_num out of bounds\n");

    if (index < 0 || index >= sizeof(uint64_t) * 8)
        panic("int_num out of bounds\n");

    interrupts[int_num] &= ~(1 << index);
    if (interrupts[int_num] == 0)
        intstatus &= ~(ULL(1) << int_num);
}

void
BaseCPU::clear_interrupts()
{
    DPRINTF(Interrupt, "Interrupts all cleared\n");

    memset(interrupts, 0, sizeof(interrupts));
    intstatus = 0;
}

#endif // FULL_SYSTEM

DEFINE_SIM_OBJECT_CLASS_NAME("BaseCPU", BaseCPU)
