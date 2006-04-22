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

#ifndef __CPU_O3_FULL_CPU_HH__
#define __CPU_O3_FULL_CPU_HH__

#include <iostream>
#include <list>
#include <queue>
#include <set>
#include <vector>

#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "config/full_system.hh"
#include "cpu/base.hh"
#include "cpu/cpu_exec_context.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/cpu_policy.hh"
#include "cpu/o3/scoreboard.hh"
#include "cpu/o3/thread_state.hh"
#include "sim/process.hh"

class ExecContext;
class MemInterface;
class Process;

class BaseFullCPU : public BaseCPU
{
    //Stuff that's pretty ISA independent will go here.
  public:
    typedef BaseCPU::Params Params;

    BaseFullCPU(Params *params);

    void regStats();

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

    typedef O3ThreadState<Impl> Thread;

    typedef typename std::list<DynInstPtr>::iterator ListIt;

  public:
    enum Status {
        Running,
        Idle,
        Halted,
        Blocked
    };

    /** Overall CPU status. */
    Status _status;

  private:
    class TickEvent : public Event
    {
      private:
        /** Pointer to the CPU. */
        FullO3CPU<Impl> *cpu;

      public:
        /** Constructs a tick event. */
        TickEvent(FullO3CPU<Impl> *c);

        /** Processes a tick event, calling tick() on the CPU. */
        void process();
        /** Returns the description of the tick event. */
        const char *description();
    };

    /** The tick event used for scheduling CPU ticks. */
    TickEvent tickEvent;

    /** Schedule tick event, regardless of its current state. */
    void scheduleTickEvent(int delay)
    {
        if (tickEvent.squashed())
            tickEvent.reschedule(curTick + delay);
        else if (!tickEvent.scheduled())
            tickEvent.schedule(curTick + delay);
    }

    /** Unschedule tick event, regardless of its current state. */
    void unscheduleTickEvent()
    {
        if (tickEvent.scheduled())
            tickEvent.squash();
    }

  public:
    /** Constructs a CPU with the given parameters. */
    FullO3CPU(Params *params);
    /** Destructor. */
    ~FullO3CPU();

    /** Registers statistics. */
    void fullCPURegStats();

    /** Ticks CPU, calling tick() on each stage, and checking the overall
     *  activity to see if the CPU should deschedule itself.
     */
    void tick();

    /** Initialize the CPU */
    void init();

    /** Setup CPU to insert a thread's context */
    void insertThread(unsigned tid);

    /** Remove all of a thread's context from CPU */
    void removeThread(unsigned tid);

    /** Count the Total Instructions Committed in the CPU. */
    virtual Counter totalInstructions() const
    {
        Counter total(0);

        for (int i=0; i < thread.size(); i++)
            total += thread[i]->numInst;

        return total;
    }

    /** Add Thread to Active Threads List. */
    void activateContext(int tid, int delay);

    /** Remove Thread from Active Threads List */
    void suspendContext(int tid);

    /** Remove Thread from Active Threads List &&
     *  Remove Thread Context from CPU.
     */
    void deallocateContext(int tid);

    /** Remove Thread from Active Threads List &&
     *  Remove Thread Context from CPU.
     */
    void haltContext(int tid);

    /** Activate a Thread When CPU Resources are Available. */
    void activateWhenReady(int tid);

    /** Add or Remove a Thread Context in the CPU. */
    void doContextSwitch();

    /** Update The Order In Which We Process Threads. */
    void updateThreadPriority();

    /** Executes a syscall on this cycle.
     *  ---------------------------------------
     *  Note: this is a virtual function. CPU-Specific
     *  functionality defined in derived classes
     */
    virtual void syscall(int tid) {}

    /** Check if there are any system calls pending. */
    void checkSyscalls();

    /** Switches out this CPU.
     *  @todo: Implement this.
     */
    void switchOut();

    /** Takes over from another CPU.
     *  @todo: Implement this.
     */
    void takeOverFrom(BaseCPU *oldCPU);

    /** Get the current instruction sequence number, and increment it. */
    InstSeqNum getAndIncrementInstSeq();

#if FULL_SYSTEM
    /** Check if this address is a valid instruction address. */
    bool validInstAddr(Addr addr) { return true; }

    /** Check if this address is a valid data address. */
    bool validDataAddr(Addr addr) { return true; }

    /** Get instruction asid. */
    int getInstAsid(unsigned tid)
    { return regFile.miscRegs[tid].getInstAsid(); }

    /** Get data asid. */
    int getDataAsid(unsigned tid)
    { return regFile.miscRegs[tid].getDataAsid(); }
#else
    /** Check if this address is a valid instruction address. */
    bool validInstAddr(Addr addr,unsigned tid)
    { return thread[tid]->validInstAddr(addr); }

    /** Check if this address is a valid data address. */
    bool validDataAddr(Addr addr,unsigned tid)
    { return thread[tid]->validDataAddr(addr); }

    /** Get instruction asid. */
    int getInstAsid(unsigned tid)
    { return thread[tid]->asid; }

    /** Get data asid. */
    int getDataAsid(unsigned tid)
    { return thread[tid]->asid; }

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

    uint64_t readArchIntReg(int reg_idx, unsigned tid);

    float readArchFloatRegSingle(int reg_idx, unsigned tid);

    double readArchFloatRegDouble(int reg_idx, unsigned tid);

    uint64_t readArchFloatRegInt(int reg_idx, unsigned tid);

    void setArchIntReg(int reg_idx, uint64_t val, unsigned tid);

    void setArchFloatRegSingle(int reg_idx, float val, unsigned tid);

    void setArchFloatRegDouble(int reg_idx, double val, unsigned tid);

    void setArchFloatRegInt(int reg_idx, uint64_t val, unsigned tid);

    uint64_t readPC(unsigned tid);

    void setPC(Addr new_PC,unsigned tid);

    uint64_t readNextPC(unsigned tid);

    void setNextPC(uint64_t val,unsigned tid);

    /** Function to add instruction onto the head of the list of the
     *  instructions.  Used when new instructions are fetched.
     */
    ListIt addInst(DynInstPtr &inst);

    /** Function to tell the CPU that an instruction has completed. */
    void instDone(unsigned tid);

    /** Add Instructions to the CPU Remove List*/
    void addToRemoveList(DynInstPtr &inst);

    /** Remove an instruction from the front of the list.  It is expected
     *  that there are no instructions in front of it (that is, none are older
     *  than the instruction being removed).  Used when retiring instructions.
     *  @todo: Remove the argument to this function, and just have it remove
     *  last instruction once it's verified that commit has the same ordering
     *  as the instruction list.
     */
    void removeFrontInst(DynInstPtr &inst);

    /** Remove all instructions that are not currently in the ROB. */
    void removeInstsNotInROB(unsigned tid);

    /** Remove all instructions younger than the given sequence number. */
    void removeInstsUntil(const InstSeqNum &seq_num,unsigned tid);

    inline void squashInstIt(const ListIt &instIt, const unsigned &tid);

    void cleanUpRemovedInsts();

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
    std::list<DynInstPtr> instList;

    /** List of all the instructions that will be removed at the end of this
     *  cycle.
     */
    std::queue<ListIt> removeList;

#ifdef DEBUG
    std::set<InstSeqNum> snList;
#endif

    /** Records if instructions need to be removed this cycle due to being
     *  retired or squashed.
     */
    bool removeInstsThisCycle;

  protected:
    /** The fetch stage. */
    typename CPUPolicy::Fetch fetch;

    /** The decode stage. */
    typename CPUPolicy::Decode decode;

    /** The dispatch stage. */
    typename CPUPolicy::Rename rename;

    /** The issue/execute/writeback stages. */
    typename CPUPolicy::IEW iew;

    /** The commit stage. */
    typename CPUPolicy::Commit commit;

    /** The register file. */
    typename CPUPolicy::RegFile regFile;

    /** The free list. */
    typename CPUPolicy::FreeList freeList;

    /** The rename map. */
    typename CPUPolicy::RenameMap renameMap[Impl::MaxThreads];

    /** The commit rename map. */
    typename CPUPolicy::RenameMap commitRenameMap[Impl::MaxThreads];

    /** The re-order buffer. */
    typename CPUPolicy::ROB rob;

    /** Active Threads List */
    std::list<unsigned> activeThreads;

    /** Integer Register Scoreboard */
    Scoreboard scoreboard;

  public:
    /** Enum to give each stage a specific index, so when calling
     *  activateStage() or deactivateStage(), they can specify which stage
     *  is being activated/deactivated.
     */
    enum StageIdx {
        FetchIdx,
        DecodeIdx,
        RenameIdx,
        IEWIdx,
        CommitIdx,
        NumStages };

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

  private:
    /** Time buffer that tracks if any cycles has active communication in them.
     *  It should be as long as the longest communication latency in the system.
     *  Each time any time buffer is written, the activity buffer should also
     *  be written to. The activityBuffer is advanced along with all the other
     *  time buffers, so it should always have a 1 somewhere in it only if there
     *  is active communication in a time buffer.
     */
    TimeBuffer<bool> activityBuffer;

    /** Tracks how many stages and cycles of time buffer have activity. Stages
     *  increment this count when they switch to active, and decrement it when
     *  they switch to inactive. Whenever a cycle that previously had no
     *  information is written in the time buffer, this is incremented. When
     *  a cycle that had information exits the time buffer due to age, this
     *  count is decremented. When the count is 0, there is no activity in the
     *  CPU, and it can be descheduled.
     */
    int activityCount;

    /** Records if there has been activity this cycle. */
    bool activity;

    /** Records which stages are active/inactive. */
    bool stageActive[NumStages];

  public:
    /** Wakes the CPU, rescheduling the CPU if it's not already active. */
    void wakeCPU();
    /** Records that there is activity this cycle. */
    void activityThisCycle();
    /** Advances the activity buffer, decrementing the activityCount if active
     *  communication just left the time buffer, and descheduling the CPU if
     *  there is no activity.
     */
    void advanceActivityBuffer();
    /** Marks a stage as active. */
    void activateStage(const StageIdx idx);
    /** Deactivates a stage. */
    void deactivateStage(const StageIdx idx);

    /** Gets a free thread id. Use if thread ids change across system. */
    int getFreeTid();

  public:
    /** Temporary function to get pointer to exec context. */
    ExecContext *xcBase(unsigned tid)
    {
        return thread[tid]->getXCProxy();
    }

    /** The global sequence number counter. */
    InstSeqNum globalSeqNum;

#if FULL_SYSTEM
    /** Pointer to the system. */
    System *system;

    /** Pointer to the memory controller. */
    MemoryController *memCtrl;
    /** Pointer to physical memory. */
    PhysicalMemory *physmem;
#endif

    // List of all ExecContexts.
    std::vector<Thread *> thread;

    /** Pointer to memory. */
    FunctionalMemory *mem;

#if 0
    /** Page table pointer. */
    PageTable *pTable;
#endif

    /** Pointer to the icache interface. */
    MemInterface *icacheInterface;
    /** Pointer to the dcache interface. */
    MemInterface *dcacheInterface;

    /** Whether or not the CPU should defer its registration. */
    bool deferRegistration;

    /** Is there a context switch pending? */
    bool contextSwitch;

    /** Threads Scheduled to Enter CPU */
    std::list<int> cpuWaitList;

    /** The cycle that the CPU was last running, used for statistics. */
    Tick lastRunningCycle;

    /** Number of Threads CPU can process */
    unsigned numThreads;

    /** Mapping for system thread id to cpu id */
    std::map<unsigned,unsigned> threadMap;

    /** Available thread ids in the cpu*/
    std::vector<unsigned> tids;

    /** Stat for total number of times the CPU is descheduled. */
    Stats::Scalar<> timesIdled;
    /** Stat for total number of cycles the CPU spends descheduled. */
    Stats::Scalar<> idleCycles;
    /** Stat for the number of committed instructions per thread. */
    Stats::Vector<> committedInsts;
    /** Stat for the total number of committed instructions. */
    Stats::Scalar<> totalCommittedInsts;
    /** Stat for the CPI per thread. */
    Stats::Formula cpi;
    /** Stat for the total CPI. */
    Stats::Formula totalCpi;
    /** Stat for the IPC per thread. */
    Stats::Formula ipc;
    /** Stat for the total IPC. */
    Stats::Formula totalIpc;
};

#endif
