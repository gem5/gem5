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

#ifndef __CPU_THREAD_STATE_HH__
#define __CPU_THREAD_STATE_HH__

#include "arch/types.hh"
#include "cpu/thread_context.hh"

#if !FULL_SYSTEM
#include "mem/mem_object.hh"
#include "mem/translating_port.hh"
#include "sim/process.hh"
#endif

#if FULL_SYSTEM
class EndQuiesceEvent;
class FunctionProfile;
class ProfileNode;
namespace Kernel {
    class Statistics;
};
#endif

class Checkpoint;

/**
 *  Struct for holding general thread state that is needed across CPU
 *  models.  This includes things such as pointers to the process,
 *  memory, quiesce events, and certain stats.  This can be expanded
 *  to hold more thread-specific stats within it.
 */
struct ThreadState {
    typedef ThreadContext::Status Status;

#if FULL_SYSTEM
    ThreadState(int _cpuId, int _tid);
#else
    ThreadState(int _cpuId, int _tid, Process *_process,
                short _asid, MemObject *mem);
#endif

    void serialize(std::ostream &os);

    void unserialize(Checkpoint *cp, const std::string &section);

    void setCpuId(int id) { cpuId = id; }

    int readCpuId() { return cpuId; }

    void setTid(int id) { tid = id; }

    int readTid() { return tid; }

    Tick readLastActivate() { return lastActivate; }

    Tick readLastSuspend() { return lastSuspend; }

#if FULL_SYSTEM
    void dumpFuncProfile();

    EndQuiesceEvent *getQuiesceEvent() { return quiesceEvent; }

    void profileClear();

    void profileSample();

    Kernel::Statistics *getKernelStats() { return kernelStats; }

    FunctionalPort *getPhysPort() { return physPort; }

    void setPhysPort(FunctionalPort *port) { physPort = port; }

    VirtualPort *getVirtPort(ThreadContext *tc = NULL) { return virtPort; }

    void setVirtPort(VirtualPort *port) { virtPort = port; }
#else
    Process *getProcessPtr() { return process; }

    TranslatingPort *getMemPort() { return port; }

    void setMemPort(TranslatingPort *_port) { port = _port; }

    int getInstAsid() { return asid; }
    int getDataAsid() { return asid; }
#endif

    /** Sets the current instruction being committed. */
    void setInst(TheISA::MachInst _inst) { inst = _inst; }

    /** Returns the current instruction being committed. */
    TheISA::MachInst getInst() { return inst; }

    /** Reads the number of instructions functionally executed and
     * committed.
     */
    Counter readFuncExeInst() { return funcExeInst; }

    /** Sets the total number of instructions functionally executed
     * and committed.
     */
    void setFuncExeInst(Counter new_val) { funcExeInst = new_val; }

    /** Returns the status of this thread. */
    Status status() const { return _status; }

    /** Sets the status of this thread. */
    void setStatus(Status new_status) { _status = new_status; }

    /** Number of instructions committed. */
    Counter numInst;
    /** Stat for number instructions committed. */
    Stats::Scalar<> numInsts;
    /** Stat for number of memory references. */
    Stats::Scalar<> numMemRefs;

    /** Number of simulated loads, used for tracking events based on
     * the number of loads committed.
     */
    Counter numLoad;

    /** The number of simulated loads committed prior to this run. */
    Counter startNumLoad;

  protected:
    ThreadContext::Status _status;

    // ID of this context w.r.t. the System or Process object to which
    // it belongs.  For full-system mode, this is the system CPU ID.
    int cpuId;

    // Index of hardware thread context on the CPU that this represents.
    int tid;

  public:
    /** Last time activate was called on this thread. */
    Tick lastActivate;

    /** Last time suspend was called on this thread. */
    Tick lastSuspend;

#if FULL_SYSTEM
  public:
    FunctionProfile *profile;
    ProfileNode *profileNode;
    Addr profilePC;
    EndQuiesceEvent *quiesceEvent;

    Kernel::Statistics *kernelStats;
  protected:
    /** A functional port outgoing only for functional accesses to physical
     * addresses.*/
    FunctionalPort *physPort;

    /** A functional port, outgoing only, for functional accesse to virtual
     * addresses. That doen't require execution context information */
    VirtualPort *virtPort;
#else
    TranslatingPort *port;

    Process *process;

    // Address space ID.  Note that this is used for TIMING cache
    // simulation only; all functional memory accesses should use
    // one of the FunctionalMemory pointers above.
    short asid;
#endif

    /** Current instruction the thread is committing.  Only set and
     * used for DTB faults currently.
     */
    TheISA::MachInst inst;

  public:
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
