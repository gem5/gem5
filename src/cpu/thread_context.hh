/*
 * Copyright (c) 2011-2012, 2016-2018, 2020 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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

#ifndef __CPU_THREAD_CONTEXT_HH__
#define __CPU_THREAD_CONTEXT_HH__

#include <iostream>
#include <string>

#include "arch/generic/htm.hh"
#include "arch/generic/isa.hh"
#include "arch/generic/pcstate.hh"
#include "base/types.hh"
#include "cpu/pc_event.hh"
#include "cpu/reg_class.hh"

namespace gem5
{

// @todo: Figure out a more architecture independent way to obtain the ITB and
// DTB pointers.
class BaseCPU;
class BaseMMU;
class BaseTLB;
class CheckerCPU;
class Checkpoint;
class InstDecoder;
class PortProxy;
class Process;
class System;
class Packet;
using PacketPtr = Packet *;

/**
 * ThreadContext is the external interface to all thread state for
 * anything outside of the CPU. It provides all accessor methods to
 * state that might be needed by external objects, ranging from
 * register values to things such as kernel stats. It is an abstract
 * base class; the CPU can create its own ThreadContext by
 * deriving from it.
 *
 * The ThreadContext is slightly different than the ExecContext.  The
 * ThreadContext provides access to an individual thread's state; an
 * ExecContext provides ISA access to the CPU (meaning it is
 * implicitly multithreaded on SMT systems).  Additionally the
 * ThreadState is an abstract class that exactly defines the
 * interface; the ExecContext is a more implicit interface that must
 * be implemented so that the ISA can access whatever state it needs.
 */
class ThreadContext : public PCEventScope
{
  protected:
    bool useForClone = false;

  public:
    bool
    getUseForClone()
    {
        return useForClone;
    }

    void
    setUseForClone(bool new_val)
    {
        useForClone = new_val;
    }

    enum Status
    {
        /// Running.  Instructions should be executed only when
        /// the context is in this state.
        Active,

        /// Temporarily inactive.  Entered while waiting for
        /// synchronization, etc.
        Suspended,

        /// Trying to exit and waiting for an event to completely exit.
        /// Entered when target executes an exit syscall.
        Halting,

        /// Permanently shut down.  Entered when target executes
        /// m5exit pseudo-instruction.  When all contexts enter
        /// this state, the simulation will terminate.
        Halted
    };

    virtual ~ThreadContext(){};

    virtual BaseCPU *getCpuPtr() = 0;

    virtual int cpuId() const = 0;

    virtual uint32_t socketId() const = 0;

    virtual int threadId() const = 0;

    virtual void setThreadId(int id) = 0;

    virtual ContextID contextId() const = 0;

    virtual void setContextId(ContextID id) = 0;

    virtual BaseMMU *getMMUPtr() = 0;

    virtual CheckerCPU *getCheckerCpuPtr() = 0;

    virtual BaseISA *getIsaPtr() const = 0;

    virtual InstDecoder *getDecoderPtr() = 0;

    virtual System *getSystemPtr() = 0;

    virtual void sendFunctional(PacketPtr pkt);

    virtual Process *getProcessPtr() = 0;

    virtual void setProcessPtr(Process *p) = 0;

    virtual Status status() const = 0;

    virtual void setStatus(Status new_status) = 0;

    /// Set the status to Active.
    virtual void activate() = 0;

    /// Set the status to Suspended.
    virtual void suspend() = 0;

    /// Set the status to Halted.
    virtual void halt() = 0;

    /// Quiesce thread context
    void quiesce();

    /// Quiesce, suspend, and schedule activate at resume
    void quiesceTick(Tick resume);

    virtual void takeOverFrom(ThreadContext *old_context) = 0;

    virtual void regStats(const std::string &name){};

    virtual void scheduleInstCountEvent(Event *event, Tick count) = 0;
    virtual void descheduleInstCountEvent(Event *event) = 0;
    virtual Tick getCurrentInstCount() = 0;

    // Not necessarily the best location for these...
    // Having an extra function just to read these is obnoxious
    virtual Tick readLastActivate() = 0;
    virtual Tick readLastSuspend() = 0;

    virtual void copyArchRegs(ThreadContext *tc) = 0;

    virtual void clearArchRegs() = 0;

    //
    // New accessors for new decoder.
    //
    virtual RegVal getReg(const RegId &reg) const;
    virtual void getReg(const RegId &reg, void *val) const = 0;
    virtual void *getWritableReg(const RegId &reg) = 0;

    virtual void setReg(const RegId &reg, RegVal val);
    virtual void setReg(const RegId &reg, const void *val) = 0;

    virtual const PCStateBase &pcState() const = 0;

    virtual void pcState(const PCStateBase &val) = 0;

    void
    pcState(Addr addr)
    {
        std::unique_ptr<PCStateBase> new_pc(getIsaPtr()->newPCState(addr));
        pcState(*new_pc);
    }

    virtual void pcStateNoRecord(const PCStateBase &val) = 0;

    virtual RegVal readMiscRegNoEffect(RegIndex misc_reg) const = 0;

    virtual RegVal readMiscReg(RegIndex misc_reg) = 0;

    virtual void setMiscRegNoEffect(RegIndex misc_reg, RegVal val) = 0;

    virtual void setMiscReg(RegIndex misc_reg, RegVal val) = 0;

    // Also not necessarily the best location for these two.  Hopefully will go
    // away once we decide upon where st cond failures goes.
    virtual unsigned readStCondFailures() const = 0;

    virtual void setStCondFailures(unsigned sc_failures) = 0;

    // This function exits the thread context in the CPU and returns
    // 1 if the CPU has no more active threads (meaning it's OK to exit);
    // Used in syscall-emulation mode when a  thread calls the exit syscall.
    virtual int
    exit()
    {
        return 1;
    };

    /** function to compare two thread contexts (for debugging) */
    static void compare(ThreadContext *one, ThreadContext *two);

    // hardware transactional memory
    virtual void htmAbortTransaction(uint64_t htm_uid,
                                     HtmFailureFaultCause cause) = 0;
    virtual BaseHTMCheckpointPtr &getHtmCheckpointPtr() = 0;
    virtual void setHtmCheckpointPtr(BaseHTMCheckpointPtr cpt) = 0;
};

/** @{ */
/**
 * Thread context serialization helpers
 *
 * These helper functions provide a way to the data in a
 * ThreadContext. They are provided as separate helper function since
 * implementing them as members of the ThreadContext interface would
 * be confusing when the ThreadContext is exported via a proxy.
 */

void serialize(const ThreadContext &tc, CheckpointOut &cp);
void unserialize(ThreadContext &tc, CheckpointIn &cp);

/** @} */

/**
 * Copy state between thread contexts in preparation for CPU handover.
 *
 * @note This method modifies the old thread contexts as well as the
 * new thread context. The old thread context will have its quiesce
 * event descheduled if it is scheduled and its status set to halted.
 *
 * @param new_tc Destination ThreadContext.
 * @param old_tc Source ThreadContext.
 */
void takeOverFrom(ThreadContext &new_tc, ThreadContext &old_tc);

} // namespace gem5

#endif
