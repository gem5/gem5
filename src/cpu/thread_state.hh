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
 */

#ifndef __CPU_THREAD_STATE_HH__
#define __CPU_THREAD_STATE_HH__

#include "cpu/thread_context.hh"

#if !FULL_SYSTEM
#include "mem/translating_port.hh"
#endif

#if FULL_SYSTEM
class EndQuiesceEvent;
class FunctionProfile;
class ProfileNode;
namespace Kernel {
    class Statistics;
};
#else
class FunctionalMemory;
class Process;
#endif

/**
 *  Struct for holding general thread state that is needed across CPU
 *  models.  This includes things such as pointers to the process,
 *  memory, quiesce events, and certain stats.  This can be expanded
 *  to hold more thread-specific stats within it.
 */
struct ThreadState {
#if FULL_SYSTEM
    ThreadState(int _cpuId, int _tid)
        : cpuId(_cpuId), tid(_tid), lastActivate(0), lastSuspend(0),
          profile(NULL), profileNode(NULL), profilePC(0), quiesceEvent(NULL)
#else
    ThreadState(int _cpuId, int _tid, MemObject *mem,
                Process *_process, short _asid)
        : cpuId(_cpuId), tid(_tid), process(_process), asid(_asid)
#endif
    {
        funcExeInst = 0;
        storeCondFailures = 0;
#if !FULL_SYSTEM
        /* Use this port to for syscall emulation writes to memory. */
        Port *mem_port;
        port = new TranslatingPort(csprintf("%d-funcport",
                                            tid),
                                   process->pTable, false);
        mem_port = mem->getPort("functional");
        mem_port->setPeer(port);
        port->setPeer(mem_port);
#endif
    }

    ThreadContext::Status status;

    int cpuId;

    // Index of hardware thread context on the CPU that this represents.
    int tid;

    Counter numInst;
    Stats::Scalar<> numInsts;
    Stats::Scalar<> numMemRefs;

    // number of simulated loads
    Counter numLoad;
    Counter startNumLoad;

#if FULL_SYSTEM
    Tick lastActivate;
    Tick lastSuspend;

    FunctionProfile *profile;
    ProfileNode *profileNode;
    Addr profilePC;

    EndQuiesceEvent *quiesceEvent;

    Kernel::Statistics *kernelStats;
#else
    TranslatingPort *port;

    Process *process;

    // Address space ID.  Note that this is used for TIMING cache
    // simulation only; all functional memory accesses should use
    // one of the FunctionalMemory pointers above.
    short asid;

#endif

    /**
     * Temporary storage to pass the source address from copy_load to
     * copy_store.
     * @todo Remove this temporary when we have a better way to do it.
     */
    Addr copySrcAddr;
    /**
     * Temp storage for the physical source address of a copy.
     * @todo Remove this temporary when we have a better way to do it.
     */
    Addr copySrcPhysAddr;

    /*
     * number of executed instructions, for matching with syscall trace
     * points in EIO files.
     */
    Counter funcExeInst;

    //
    // Count failed store conditionals so we can warn of apparent
    // application deadlock situations.
    unsigned storeCondFailures;
};

#endif // __CPU_THREAD_STATE_HH__
