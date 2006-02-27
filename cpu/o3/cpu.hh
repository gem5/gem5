/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

//Todo: Add in a lot of the functions that are ISA specific.  Also define
//the functions that currently exist within the base cpu class.  Define
//everything for the simobject stuff so it can be serialized and
//instantiated, add in debugging statements everywhere.  Have CPU schedule
//itself properly.  Threads!
// Avoid running stages and advancing queues if idle/stalled.

#ifndef __CPU_O3_CPU_FULL_CPU_HH__
#define __CPU_O3_CPU_FULL_CPU_HH__

#include <iostream>
#include <list>
#include <vector>

#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "config/full_system.hh"
#include "cpu/base.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/cpu_policy.hh"
#include "cpu/exec_context.hh"
#include "sim/process.hh"

#if FULL_SYSTEM
#include "arch/ev5.hh"
using namespace EV5;
#endif

class FunctionalMemory;
class Process;

class BaseFullCPU : public BaseCPU
{
    //Stuff that's pretty ISA independent will go here.
  public:
    typedef BaseCPU::Params Params;

#if FULL_SYSTEM
    BaseFullCPU(Params &params);
#else
    BaseFullCPU(Params &params);
#endif // FULL_SYSTEM

  protected:
    int cpu_id;
};

template <class Impl>
class FullO3CPU : public BaseFullCPU
{
  public:
    //Put typedefs from the Impl here.
    typedef typename Impl::CPUPol CPUPolicy;
    typedef typename Impl::Params Params;
    typedef typename Impl::DynInstPtr DynInstPtr;

  public:
    enum Status {
        Running,
        Idle,
        Halted,
        Blocked // ?
    };

    Status _status;

  private:
    class TickEvent : public Event
    {
      private:
        FullO3CPU<Impl> *cpu;

      public:
        TickEvent(FullO3CPU<Impl> *c);
        void process();
        const char *description();
    };

    TickEvent tickEvent;

    /// Schedule tick event, regardless of its current state.
    void scheduleTickEvent(int delay)
    {
        if (tickEvent.squashed())
            tickEvent.reschedule(curTick + delay);
        else if (!tickEvent.scheduled())
            tickEvent.schedule(curTick + delay);
    }

    /// Unschedule tick event, regardless of its current state.
    void unscheduleTickEvent()
    {
        if (tickEvent.scheduled())
            tickEvent.squash();
    }

  public:
    FullO3CPU(Params &params);
    ~FullO3CPU();

    void fullCPURegStats();

    void tick();

    void init();

    void activateContext(int thread_num, int delay);
    void suspendContext(int thread_num);
    void deallocateContext(int thread_num);
    void haltContext(int thread_num);

    void switchOut();
    void takeOverFrom(BaseCPU *oldCPU);

    /** Get the current instruction sequence number, and increment it. */
    InstSeqNum getAndIncrementInstSeq();

#if FULL_SYSTEM
    /** Check if this address is a valid instruction address. */
    bool validInstAddr(Addr addr) { return true; }

    /** Check if this address is a valid data address. */
    bool validDataAddr(Addr addr) { return true; }

    /** Get instruction asid. */
    int getInstAsid()
    { return ITB_ASN_ASN(regFile.getIpr()[TheISA::IPR_ITB_ASN]); }

    /** Get data asid. */
    int getDataAsid()
    { return DTB_ASN_ASN(regFile.getIpr()[TheISA::IPR_DTB_ASN]); }
#else
    bool validInstAddr(Addr addr)
    { return thread[0]->validInstAddr(addr); }

    bool validDataAddr(Addr addr)
    { return thread[0]->validDataAddr(addr); }

    int getInstAsid() { return thread[0]->asid; }
    int getDataAsid() { return thread[0]->asid; }

#endif

    //
    // New accessors for new decoder.
    //
    uint64_t readIntReg(int reg_idx);

    float readFloatRegSingle(int reg_idx);

    double readFloatRegDouble(int reg_idx);

    uint64_t readFloatRegInt(int reg_idx);

    void setIntReg(int reg_idx, uint64_t val);

    void setFloatRegSingle(int reg_idx, float val);

    void setFloatRegDouble(int reg_idx, double val);

    void setFloatRegInt(int reg_idx, uint64_t val);

    uint64_t readPC();

    void setNextPC(uint64_t val);

    void setPC(Addr new_PC);

    /** Function to add instruction onto the head of the list of the
     *  instructions.  Used when new instructions are fetched.
     */
    void addInst(DynInstPtr &inst);

    /** Function to tell the CPU that an instruction has completed. */
    void instDone();

    /** Remove all instructions in back of the given instruction, but leave
     *  that instruction in the list.  This is useful in a squash, when there
     *  are instructions in this list that don't exist in structures such as
     *  the ROB.  The instruction doesn't have to be the last instruction in
     *  the list, but will be once this function completes.
     *  @todo: Remove only up until that inst?  Squashed inst is most likely
     *  valid.
     */
    void removeBackInst(DynInstPtr &inst);

    /** Remove an instruction from the front of the list.  It is expected
     *  that there are no instructions in front of it (that is, none are older
     *  than the instruction being removed).  Used when retiring instructions.
     *  @todo: Remove the argument to this function, and just have it remove
     *  last instruction once it's verified that commit has the same ordering
     *  as the instruction list.
     */
    void removeFrontInst(DynInstPtr &inst);

    /** Remove all instructions that are not currently in the ROB. */
    void removeInstsNotInROB();

    /** Remove all instructions younger than the given sequence number. */
    void removeInstsUntil(const InstSeqNum &seq_num);

    /** Remove all instructions from the list. */
    void removeAllInsts();

    void dumpInsts();

    /** Basically a wrapper function so that instructions executed at
     *  commit can tell the instruction queue that they have completed.
     *  Eventually this hack should be removed.
     */
    void wakeDependents(DynInstPtr &inst);

  public:
    /** List of all the instructions in flight. */
    list<DynInstPtr> instList;

    //not sure these should be private.
  protected:
    /** The fetch stage. */
    typename CPUPolicy::Fetch fetch;

    /** The fetch stage's status. */
    typename CPUPolicy::Fetch::Status fetchStatus;

    /** The decode stage. */
    typename CPUPolicy::Decode decode;

    /** The decode stage's status. */
    typename CPUPolicy::Decode::Status decodeStatus;

    /** The dispatch stage. */
    typename CPUPolicy::Rename rename;

    /** The dispatch stage's status. */
    typename CPUPolicy::Rename::Status renameStatus;

    /** The issue/execute/writeback stages. */
    typename CPUPolicy::IEW iew;

    /** The issue/execute/writeback stage's status. */
    typename CPUPolicy::IEW::Status iewStatus;

    /** The commit stage. */
    typename CPUPolicy::Commit commit;

    /** The fetch stage's status. */
    typename CPUPolicy::Commit::Status commitStatus;

    //Might want to just pass these objects in to the constructors of the
    //appropriate stage.  regFile is in iew, freeList in dispatch, renameMap
    //in dispatch, and the rob in commit.
    /** The register file. */
    typename CPUPolicy::RegFile regFile;

    /** The free list. */
    typename CPUPolicy::FreeList freeList;

    /** The rename map. */
    typename CPUPolicy::RenameMap renameMap;

    /** The re-order buffer. */
    typename CPUPolicy::ROB rob;

  public:
    /** Typedefs from the Impl to get the structs that each of the
     *  time buffers should use.
     */
    typedef typename CPUPolicy::TimeStruct TimeStruct;

    typedef typename CPUPolicy::FetchStruct FetchStruct;

    typedef typename CPUPolicy::DecodeStruct DecodeStruct;

    typedef typename CPUPolicy::RenameStruct RenameStruct;

    typedef typename CPUPolicy::IEWStruct IEWStruct;

    /** The main time buffer to do backwards communication. */
    TimeBuffer<TimeStruct> timeBuffer;

    /** The fetch stage's instruction queue. */
    TimeBuffer<FetchStruct> fetchQueue;

    /** The decode stage's instruction queue. */
    TimeBuffer<DecodeStruct> decodeQueue;

    /** The rename stage's instruction queue. */
    TimeBuffer<RenameStruct> renameQueue;

    /** The IEW stage's instruction queue. */
    TimeBuffer<IEWStruct> iewQueue;

  public:
    /** The temporary exec context to support older accessors. */
    ExecContext *xc;

    /** Temporary function to get pointer to exec context. */
    ExecContext *xcBase()
    {
#if FULL_SYSTEM
        return system->execContexts[0];
#else
        return thread[0];
#endif
    }

    InstSeqNum globalSeqNum;

#if FULL_SYSTEM
    System *system;

    MemoryController *memCtrl;
    PhysicalMemory *physmem;

    AlphaITB *itb;
    AlphaDTB *dtb;

//    SWContext *swCtx;
#else
    std::vector<ExecContext *> thread;
#endif

    FunctionalMemory *mem;

    MemInterface *icacheInterface;
    MemInterface *dcacheInterface;

    bool deferRegistration;

    Counter numInsts;

    Counter funcExeInst;
};

#endif
