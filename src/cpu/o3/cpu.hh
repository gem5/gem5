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
 *
 * Authors: Kevin Lim
 */

#ifndef __CPU_O3_CPU_HH__
#define __CPU_O3_CPU_HH__

#include <iostream>
#include <list>
#include <queue>
#include <set>
#include <vector>

#include "arch/isa_traits.hh"
#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "config/full_system.hh"
#include "cpu/activity.hh"
#include "cpu/base.hh"
#include "cpu/simple_thread.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/cpu_policy.hh"
#include "cpu/o3/scoreboard.hh"
#include "cpu/o3/thread_state.hh"
#include "sim/process.hh"

template <class>
class Checker;
class ThreadContext;
class MemObject;
class Process;

class BaseFullCPU : public BaseCPU
{
    //Stuff that's pretty ISA independent will go here.
  public:
    typedef BaseCPU::Params Params;

    BaseFullCPU(Params *params);

    void regStats();

    int readCpuId() { return cpu_id; }

  protected:
    int cpu_id;
};

/**
 * FullO3CPU class, has each of the stages (fetch through commit)
 * within it, as well as all of the time buffers between stages.  The
 * tick() function for the CPU is defined here.
 */
template <class Impl>
class FullO3CPU : public BaseFullCPU
{
  public:
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;

    // Typedefs from the Impl here.
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
        Blocked,
        SwitchedOut
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
            tickEvent.reschedule(curTick + cycles(delay));
        else if (!tickEvent.scheduled())
            tickEvent.schedule(curTick + cycles(delay));
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
    virtual void syscall(int tid) { panic("Unimplemented!"); }

    /** Switches out this CPU. */
    void switchOut(Sampler *sampler);

    /** Signals to this CPU that a stage has completed switching out. */
    void signalSwitched();

    /** Takes over from another CPU. */
    void takeOverFrom(BaseCPU *oldCPU);

    /** Get the current instruction sequence number, and increment it. */
    InstSeqNum getAndIncrementInstSeq()
    { return globalSeqNum++; }

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
    /** Get instruction asid. */
    int getInstAsid(unsigned tid)
    { return thread[tid]->getInstAsid(); }

    /** Get data asid. */
    int getDataAsid(unsigned tid)
    { return thread[tid]->getDataAsid(); }

#endif

    /** Register accessors.  Index refers to the physical register index. */
    uint64_t readIntReg(int reg_idx);

    FloatReg readFloatReg(int reg_idx);

    FloatReg readFloatReg(int reg_idx, int width);

    FloatRegBits readFloatRegBits(int reg_idx);

    FloatRegBits readFloatRegBits(int reg_idx, int width);

    void setIntReg(int reg_idx, uint64_t val);

    void setFloatReg(int reg_idx, FloatReg val);

    void setFloatReg(int reg_idx, FloatReg val, int width);

    void setFloatRegBits(int reg_idx, FloatRegBits val);

    void setFloatRegBits(int reg_idx, FloatRegBits val, int width);

    uint64_t readArchIntReg(int reg_idx, unsigned tid);

    float readArchFloatRegSingle(int reg_idx, unsigned tid);

    double readArchFloatRegDouble(int reg_idx, unsigned tid);

    uint64_t readArchFloatRegInt(int reg_idx, unsigned tid);

    /** Architectural register accessors.  Looks up in the commit
     * rename table to obtain the true physical index of the
     * architected register first, then accesses that physical
     * register.
     */
    void setArchIntReg(int reg_idx, uint64_t val, unsigned tid);

    void setArchFloatRegSingle(int reg_idx, float val, unsigned tid);

    void setArchFloatRegDouble(int reg_idx, double val, unsigned tid);

    void setArchFloatRegInt(int reg_idx, uint64_t val, unsigned tid);

    /** Reads the commit PC of a specific thread. */
    uint64_t readPC(unsigned tid);

    /** Sets the commit PC of a specific thread. */
    void setPC(Addr new_PC, unsigned tid);

    /** Reads the next PC of a specific thread. */
    uint64_t readNextPC(unsigned tid);

    /** Sets the next PC of a specific thread. */
    void setNextPC(uint64_t val, unsigned tid);

    /** Function to add instruction onto the head of the list of the
     *  instructions.  Used when new instructions are fetched.
     */
    ListIt addInst(DynInstPtr &inst);

    /** Function to tell the CPU that an instruction has completed. */
    void instDone(unsigned tid);

    /** Add Instructions to the CPU Remove List*/
    void addToRemoveList(DynInstPtr &inst);

    /** Remove an instruction from the front end of the list.  There's
     *  no restriction on location of the instruction.
     */
    void removeFrontInst(DynInstPtr &inst);

    /** Remove all instructions that are not currently in the ROB. */
    void removeInstsNotInROB(unsigned tid);

    /** Remove all instructions younger than the given sequence number. */
    void removeInstsUntil(const InstSeqNum &seq_num,unsigned tid);

    /** Removes the instruction pointed to by the iterator. */
    inline void squashInstIt(const ListIt &instIt, const unsigned &tid);

    /** Cleans up all instructions on the remove list. */
    void cleanUpRemovedInsts();

    /** Debug function to print all instructions on the list. */
    void dumpInsts();

  public:
    /** List of all the instructions in flight. */
    std::list<DynInstPtr> instList;

    /** List of all the instructions that will be removed at the end of this
     *  cycle.
     */
    std::queue<ListIt> removeList;

#ifdef DEBUG
    /** Debug structure to keep track of the sequence numbers still in
     * flight.
     */
    std::set<InstSeqNum> snList;
#endif

    /** Records if instructions need to be removed this cycle due to
     *  being retired or squashed.
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
    /** The activity recorder; used to tell if the CPU has any
     * activity remaining or if it can go to idle and deschedule
     * itself.
     */
    ActivityRecorder activityRec;

  public:
    /** Records that there was time buffer activity this cycle. */
    void activityThisCycle() { activityRec.activity(); }

    /** Changes a stage's status to active within the activity recorder. */
    void activateStage(const StageIdx idx)
    { activityRec.activateStage(idx); }

    /** Changes a stage's status to inactive within the activity recorder. */
    void deactivateStage(const StageIdx idx)
    { activityRec.deactivateStage(idx); }

    /** Wakes the CPU, rescheduling the CPU if it's not already active. */
    void wakeCPU();

    /** Gets a free thread id. Use if thread ids change across system. */
    int getFreeTid();

  public:
    /** Returns a pointer to a thread context. */
    ThreadContext *tcBase(unsigned tid)
    {
        return thread[tid]->getTC();
    }

    /** The global sequence number counter. */
    InstSeqNum globalSeqNum;

    /** Pointer to the checker, which can dynamically verify
     * instruction results at run time.  This can be set to NULL if it
     * is not being used.
     */
    Checker<DynInstPtr> *checker;

#if FULL_SYSTEM
    /** Pointer to the system. */
    System *system;

    /** Pointer to physical memory. */
    PhysicalMemory *physmem;
#endif

    /** Pointer to memory. */
    MemObject *mem;

    /** Pointer to the sampler */
    Sampler *sampler;

    /** Counter of how many stages have completed switching out. */
    int switchCount;

    /** Pointers to all of the threads in the CPU. */
    std::vector<Thread *> thread;

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

#endif // __CPU_O3_CPU_HH__
