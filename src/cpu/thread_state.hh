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

#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "sim/process.hh"

namespace gem5
{

class Checkpoint;

/**
 *  Struct for holding general thread state that is needed across CPU
 *  models.  This includes things such as pointers to the process,
 *  memory, quiesce events, and certain stats.  This can be expanded
 *  to hold more thread-specific stats within it.
 */
struct ThreadState : public Serializable
{
    typedef ThreadContext::Status Status;

    ThreadState(BaseCPU *cpu, ThreadID _tid, Process *_process);

    virtual ~ThreadState() = default;

    void serialize(CheckpointOut &cp) const override;

    void unserialize(CheckpointIn &cp) override;

    int
    cpuId() const
    {
        return baseCpu->cpuId();
    }

    uint32_t
    socketId() const
    {
        return baseCpu->socketId();
    }

    ContextID
    contextId() const
    {
        return _contextId;
    }

    void
    setContextId(ContextID id)
    {
        _contextId = id;
    }

    void
    setThreadId(ThreadID id)
    {
        _threadId = id;
    }

    ThreadID
    threadId() const
    {
        return _threadId;
    }

    Tick
    readLastActivate() const
    {
        return lastActivate;
    }

    Tick
    readLastSuspend() const
    {
        return lastSuspend;
    }

    Process *
    getProcessPtr()
    {
        return process;
    }

    void
    setProcessPtr(Process *p)
    {
        process = p;
    }

    /** Returns the status of this thread. */
    Status
    status() const
    {
        return _status;
    }

    /** Sets the status of this thread. */
    void
    setStatus(Status new_status)
    {
        _status = new_status;
    }

  public:
    /** Number of instructions committed. */
    Counter numInst;
    /** Number of ops (including micro ops) committed. */
    Counter numOp;

    // Defining the stat group
    struct ThreadStateStats : public statistics::Group
    {
        ThreadStateStats(BaseCPU *cpu, const ThreadID &thread);
        /** Stat for number instructions committed. */
        statistics::Scalar numInsts;
        /** Stat for number ops (including micro ops) committed. */
        statistics::Scalar numOps;
        /** Stat for number of memory references. */
        statistics::Scalar numMemRefs;
    } threadStats;

    /** Number of simulated loads, used for tracking events based on
     * the number of loads committed.
     */
    Counter numLoad;

    /** The number of simulated loads committed prior to this run. */
    Counter startNumLoad;

  protected:
    ThreadContext::Status _status;

    // Pointer to the base CPU.
    BaseCPU *baseCpu;

    // system wide HW context id
    ContextID _contextId;

    // Index of hardware thread context on the CPU that this represents.
    ThreadID _threadId;

  public:
    /** Last time activate was called on this thread. */
    Tick lastActivate;

    /** Last time suspend was called on this thread. */
    Tick lastSuspend;

  protected:
    Process *process;

  public:
    //
    // Count failed store conditionals so we can warn of apparent
    // application deadlock situations.
    unsigned storeCondFailures;
};

} // namespace gem5

#endif // __CPU_THREAD_STATE_HH__
