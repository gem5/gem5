/*
 * Copyright 2018 Google, Inc.
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
 * Authors: Gabe Black
 */

#ifndef __SYSTEMC_CORE_SCHEDULER_HH__
#define __SYSTEMC_CORE_SCHEDULER_HH__

#include "systemc/core/list.hh"
#include "systemc/core/process.hh"

namespace sc_gem5
{

typedef NodeList<Process> ProcessList;

class Scheduler
{
  public:
    Scheduler();

    uint64_t numCycles() { return _numCycles; }
    Process *current() { return _current; }

    // Run the initialization phase.
    void initialize();

    // Run delta cycles until time needs to advance.
    void runCycles();

    // Put a process on the list of processes to be initialized.
    void init(Process *p) { initList.pushLast(p); }

    // Run the next process, if there is one.
    void yield();

    // Put a process on the ready list.
    void
    ready(Process *p)
    {
        // Clump methods together to minimize context switching.
        if (p->procKind() == ::sc_core::SC_METHOD_PROC_)
            readyList.pushFirst(p);
        else
            readyList.pushLast(p);
    }

    // Run the given process immediately, preempting whatever may be running.
    void
    runNow(Process *p)
    {
        // If a process is running, schedule it/us to run again.
        if (_current)
            readyList.pushFirst(_current);
        // Schedule p to run first.
        readyList.pushFirst(p);
        yield();
    }

  private:
    uint64_t _numCycles;

    Process *_current;

    ProcessList initList;
    ProcessList readyList;

    void evaluate();
    void update();
    void delta();
};

extern Scheduler scheduler;

} // namespace sc_gem5

#endif // __SYSTEMC_CORE_SCHEDULER_H__
