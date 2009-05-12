/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Korey Sewell
 *
 */

#ifndef __CPU_INORDER_CPU_HH__
#define __CPU_INORDER_CPU_HH__

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
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/pipeline_stage.hh"
#include "cpu/inorder/thread_state.hh"
#include "cpu/inorder/reg_dep_map.hh"
#include "cpu/o3/dep_graph.hh"
#include "cpu/o3/rename_map.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/eventq.hh"
#include "sim/process.hh"

class ThreadContext;
class MemInterface;
class MemObject;
class Process;
class ResourcePool;

class InOrderCPU : public BaseCPU
{

  protected:
    typedef ThePipeline::Params Params;
    typedef InOrderThreadState Thread;

   //ISA TypeDefs
    typedef TheISA::IntReg IntReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
    typedef TheISA::MiscReg MiscReg;
    typedef TheISA::RegFile RegFile;

    //DynInstPtr TypeDefs
    typedef ThePipeline::DynInstPtr DynInstPtr;
    typedef std::list<DynInstPtr>::iterator ListIt;

    //TimeBuffer TypeDefs
    typedef TimeBuffer<InterStageStruct> StageQueue;

    friend class Resource;

  public:
    /** Constructs a CPU with the given parameters. */
    InOrderCPU(Params *params);

    /** CPU ID */
    int cpu_id;

    /** Type of core that this is */
    std::string coreType;

    int readCpuId() { return cpu_id; }

    void setCpuId(int val) { cpu_id = val; }

    Params *cpu_params;

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
    /** Define TickEvent for the CPU */
    class TickEvent : public Event
    {
      private:
        /** Pointer to the CPU. */
        InOrderCPU *cpu;

      public:
        /** Constructs a tick event. */
        TickEvent(InOrderCPU *c);

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
          mainEventQueue.reschedule(&tickEvent, nextCycle(curTick + ticks(delay)));
        else if (!tickEvent.scheduled())
          mainEventQueue.schedule(&tickEvent, nextCycle(curTick + ticks(delay)));
    }

    /** Unschedule tick event, regardless of its current state. */
    void unscheduleTickEvent()
    {
        if (tickEvent.scheduled())
            tickEvent.squash();
    }

  public:
    // List of Events That can be scheduled from
    // within the CPU.
    // NOTE(1): The Resource Pool also uses this event list
    // to schedule events broadcast to all resources interfaces
    // NOTE(2): CPU Events usually need to schedule a corresponding resource
    // pool event.
    enum CPUEventType {
        ActivateThread,
        DeallocateThread,
        SuspendThread,
        DisableThreads,
        EnableThreads,
        DisableVPEs,
        EnableVPEs,
        Trap,
        InstGraduated,
        SquashAll,
        UpdatePCs,
        NumCPUEvents
    };

    /** Define CPU Event */
    class CPUEvent : public Event
    {
      protected:
        InOrderCPU *cpu;

      public:
        CPUEventType cpuEventType;
        unsigned tid;
        unsigned vpe;
        Fault fault;

      public:
        /** Constructs a CPU event. */
        CPUEvent(InOrderCPU *_cpu, CPUEventType e_type, Fault fault,
                 unsigned _tid, unsigned _vpe);

        /** Set Type of Event To Be Scheduled */
        void setEvent(CPUEventType e_type, Fault _fault, unsigned _tid, unsigned _vpe)
        {
            fault = _fault;
            cpuEventType = e_type;
            tid = _tid;
            vpe = _vpe;
        }

        /** Processes a resource event. */
        virtual void process();

        /** Returns the description of the resource event. */
        const char *description();

        /** Schedule Event */
        void scheduleEvent(int delay);

        /** Unschedule This Event */
        void unscheduleEvent();
    };

    /** Schedule a CPU Event */
    void scheduleCpuEvent(CPUEventType cpu_event, Fault fault, unsigned tid,
                          unsigned vpe, unsigned delay = 0);

  public:
    /** Interface between the CPU and CPU resources. */
    ResourcePool *resPool;

    /** Instruction used to signify that there is no *real* instruction in buffer slot */
    DynInstPtr dummyBufferInst;

    /** Used by resources to signify a denied access to a resource. */
    ResourceRequest *dummyReq;

    /** Identifies the resource id that identifies a fetch
     * access unit.
     */
    unsigned fetchPortIdx;

    /** Identifies the resource id that identifies a ITB       */
    unsigned itbIdx;

    /** Identifies the resource id that identifies a data
     * access unit.
     */
    unsigned dataPortIdx;

    /** Identifies the resource id that identifies a DTB       */
    unsigned dtbIdx;

    /** The Pipeline Stages for the CPU */
    PipelineStage *pipelineStage[ThePipeline::NumStages];

    /** Program Counters */
    TheISA::IntReg PC[ThePipeline::MaxThreads];
    TheISA::IntReg nextPC[ThePipeline::MaxThreads];
    TheISA::IntReg nextNPC[ThePipeline::MaxThreads];

    /** The Register File for the CPU */
    TheISA::IntRegFile intRegFile[ThePipeline::MaxThreads];;
    TheISA::FloatRegFile floatRegFile[ThePipeline::MaxThreads];;
    TheISA::MiscRegFile miscRegFile;

    /** Dependency Tracker for Integer & Floating Point Regs */
    RegDepMap archRegDepMap[ThePipeline::MaxThreads];

    /** Global communication structure */
    TimeBuffer<TimeStruct> timeBuffer;

    /** Communication structure that sits in between pipeline stages */
    StageQueue *stageQueue[ThePipeline::NumStages-1];

    TheISA::ITB *getITBPtr();
    TheISA::DTB *getDTBPtr();

  public:

    /** Registers statistics. */
    void regStats();

    /** Ticks CPU, calling tick() on each stage, and checking the overall
     *  activity to see if the CPU should deschedule itself.
     */
    void tick();

    /** Initialize the CPU */
    void init();

    /** Reset State in the CPU */
    void reset();

    /** Get a Memory Port */
    Port* getPort(const std::string &if_name, int idx = 0);

    /** trap() - sets up a trap event on the cpuTraps to handle given fault.
     *  trapCPU() - Traps to handle given fault
     */
    void trap(Fault fault, unsigned tid, int delay = 0);
    void trapCPU(Fault fault, unsigned tid);

    /** Setup CPU to insert a thread's context */
    void insertThread(unsigned tid);

    /** Remove all of a thread's context from CPU */
    void removeThread(unsigned tid);

    /** Add Thread to Active Threads List. */
    void activateContext(unsigned tid, int delay = 0);
    void activateThread(unsigned tid);

    /** Remove Thread from Active Threads List */
    void suspendContext(unsigned tid, int delay = 0);
    void suspendThread(unsigned tid);

    /** Remove Thread from Active Threads List &&
     *  Remove Thread Context from CPU.
     */
    void deallocateContext(unsigned tid, int delay = 0);
    void deallocateThread(unsigned tid);
    void deactivateThread(unsigned tid);

    PipelineStage* getPipeStage(int stage_num);

    int
    contextId()
    {
        hack_once("return a bogus context id");
        return 0;
    }

    /** Remove Thread from Active Threads List &&
     *  Remove Thread Context from CPU.
     */
    void haltContext(unsigned tid, int delay = 0);

    void removePipelineStalls(unsigned tid);

    void squashThreadInPipeline(unsigned tid);

    /// Notify the CPU to enable a virtual processor element.
    virtual void enableVirtProcElement(unsigned vpe);
    void enableVPEs(unsigned vpe);

    /// Notify the CPU to disable a virtual processor element.
    virtual void disableVirtProcElement(unsigned tid, unsigned vpe);
    void disableVPEs(unsigned tid, unsigned vpe);

    /// Notify the CPU that multithreading is enabled.
    virtual void enableMultiThreading(unsigned vpe);
    void enableThreads(unsigned vpe);

    /// Notify the CPU that multithreading is disabled.
    virtual void disableMultiThreading(unsigned tid, unsigned vpe);
    void disableThreads(unsigned tid, unsigned vpe);

    /** Activate a Thread When CPU Resources are Available. */
    void activateWhenReady(int tid);

    /** Add or Remove a Thread Context in the CPU. */
    void doContextSwitch();

    /** Update The Order In Which We Process Threads. */
    void updateThreadPriority();

    /** Switches a Pipeline Stage to Active. (Unused currently) */
    void switchToActive(int stage_idx)
    { /*pipelineStage[stage_idx]->switchToActive();*/ }

    /** Get the current instruction sequence number, and increment it. */
    InstSeqNum getAndIncrementInstSeq(unsigned tid)
    { return globalSeqNum[tid]++; }

    /** Get the current instruction sequence number, and increment it. */
    InstSeqNum nextInstSeqNum(unsigned tid)
    { return globalSeqNum[tid]; }

    /** Increment Instruction Sequence Number */
    void incrInstSeqNum(unsigned tid)
    { globalSeqNum[tid]++; }

    /** Set Instruction Sequence Number */
    void setInstSeqNum(unsigned tid, InstSeqNum seq_num)
    {
        globalSeqNum[tid] = seq_num;
    }

    /** Get & Update Next Event Number */
    InstSeqNum getNextEventNum()
    {
        return cpuEventNum++;
    }

    /** Get instruction asid. */
    int getInstAsid(unsigned tid)
    { return thread[tid]->getInstAsid(); }

    /** Get data asid. */
    int getDataAsid(unsigned tid)
    { return thread[tid]->getDataAsid(); }

    /** Register file accessors  */
    uint64_t readIntReg(int reg_idx, unsigned tid);

    FloatReg readFloatReg(int reg_idx, unsigned tid,
                          int width = TheISA::SingleWidth);

    FloatRegBits readFloatRegBits(int reg_idx, unsigned tid,
                                  int width = TheISA::SingleWidth);

    void setIntReg(int reg_idx, uint64_t val, unsigned tid);

    void setFloatReg(int reg_idx, FloatReg val,  unsigned tid,
                     int width = TheISA::SingleWidth);

    void setFloatRegBits(int reg_idx, FloatRegBits val,  unsigned tid,
                         int width = TheISA::SingleWidth);

    /** Reads a miscellaneous register. */
    MiscReg readMiscRegNoEffect(int misc_reg, unsigned tid = 0);

    /** Reads a misc. register, including any side effects the read
     * might have as defined by the architecture.
     */
    MiscReg readMiscReg(int misc_reg, unsigned tid = 0);

    /** Sets a miscellaneous register. */
    void setMiscRegNoEffect(int misc_reg, const MiscReg &val, unsigned tid = 0);

    /** Sets a misc. register, including any side effects the write
     * might have as defined by the architecture.
     */
    void setMiscReg(int misc_reg, const MiscReg &val, unsigned tid = 0);

    /** Reads a int/fp/misc reg. from another thread depending on ISA-defined
     *  target thread
     */
    uint64_t readRegOtherThread(unsigned misc_reg, unsigned tid = -1);

    /** Sets a int/fp/misc reg. from another thread depending on an ISA-defined
     * target thread
     */
    void setRegOtherThread(unsigned misc_reg, const MiscReg &val, unsigned tid);

    /** Reads the commit PC of a specific thread. */
    uint64_t readPC(unsigned tid);

    /** Sets the commit PC of a specific thread. */
    void setPC(Addr new_PC, unsigned tid);

    /** Reads the next PC of a specific thread. */
    uint64_t readNextPC(unsigned tid);

    /** Sets the next PC of a specific thread. */
    void setNextPC(uint64_t val, unsigned tid);

    /** Reads the next NPC of a specific thread. */
    uint64_t readNextNPC(unsigned tid);

    /** Sets the next NPC of a specific thread. */
    void setNextNPC(uint64_t val, unsigned tid);

    /** Function to add instruction onto the head of the list of the
     *  instructions.  Used when new instructions are fetched.
     */
    ListIt addInst(DynInstPtr &inst);

    /** Function to tell the CPU that an instruction has completed. */
    void instDone(DynInstPtr inst, unsigned tid);

    /** Add Instructions to the CPU Remove List*/
    void addToRemoveList(DynInstPtr &inst);

    /** Remove an instruction from CPU */
    void removeInst(DynInstPtr &inst);

    /** Remove all instructions younger than the given sequence number. */
    void removeInstsUntil(const InstSeqNum &seq_num,unsigned tid);

    /** Removes the instruction pointed to by the iterator. */
    inline void squashInstIt(const ListIt &instIt, const unsigned &tid);

    /** Cleans up all instructions on the instruction remove list. */
    void cleanUpRemovedInsts();

    /** Cleans up all instructions on the request remove list. */
    void cleanUpRemovedReqs();

    /** Cleans up all instructions on the CPU event remove list. */
    void cleanUpRemovedEvents();

    /** Debug function to print all instructions on the list. */
    void dumpInsts();

    /** Forwards an instruction read to the appropriate data
     *  resource (indexes into Resource Pool thru "dataPortIdx")
     */
    Fault read(DynInstPtr inst);

    /** Forwards an instruction write. to the appropriate data
     *  resource (indexes into Resource Pool thru "dataPortIdx")
     */
    Fault write(DynInstPtr inst);

    /** Executes a syscall.*/
    void syscall(int64_t callnum, int tid);

  public:
    /** Per-Thread List of all the instructions in flight. */
    std::list<DynInstPtr> instList[ThePipeline::MaxThreads];

    /** List of all the instructions that will be removed at the end of this
     *  cycle.
     */
    std::queue<ListIt> removeList;

    /** List of all the resource requests that will be removed at the end of this
     *  cycle.
     */
    std::queue<ResourceRequest*> reqRemoveList;

    /** List of all the cpu event requests that will be removed at the end of
     *  the current cycle.
     */
    std::queue<Event*> cpuEventRemoveList;

    /** Records if instructions need to be removed this cycle due to
     *  being retired or squashed.
     */
    bool removeInstsThisCycle;

    /** True if there is non-speculative Inst Active In Pipeline. Lets any
     * execution unit know, NOT to execute while the instruction is active.
     */
    bool nonSpecInstActive[ThePipeline::MaxThreads];

    /** Instruction Seq. Num of current non-speculative instruction. */
    InstSeqNum nonSpecSeqNum[ThePipeline::MaxThreads];

    /** Instruction Seq. Num of last instruction squashed in pipeline */
    InstSeqNum squashSeqNum[ThePipeline::MaxThreads];

    /** Last Cycle that the CPU squashed instruction end. */
    Tick lastSquashCycle[ThePipeline::MaxThreads];

    std::list<unsigned> fetchPriorityList;

  protected:
    /** Active Threads List */
    std::list<unsigned> activeThreads;

    /** Current Threads List */
    std::list<unsigned> currentThreads;

    /** Suspended Threads List */
    std::list<unsigned> suspendedThreads;

    /** Thread Status Functions (Unused Currently) */
    bool isThreadInCPU(unsigned tid);
    bool isThreadActive(unsigned tid);
    bool isThreadSuspended(unsigned tid);
    void addToCurrentThreads(unsigned tid);
    void removeFromCurrentThreads(unsigned tid);

  private:
    /** The activity recorder; used to tell if the CPU has any
     * activity remaining or if it can go to idle and deschedule
     * itself.
     */
    ActivityRecorder activityRec;

  public:
    void readFunctional(Addr addr, uint32_t &buffer);

    /** Number of Active Threads in the CPU */
    int numActiveThreads() { return activeThreads.size(); }

    /** Records that there was time buffer activity this cycle. */
    void activityThisCycle() { activityRec.activity(); }

    /** Changes a stage's status to active within the activity recorder. */
    void activateStage(const int idx)
    { activityRec.activateStage(idx); }

    /** Changes a stage's status to inactive within the activity recorder. */
    void deactivateStage(const int idx)
    { activityRec.deactivateStage(idx); }

    /** Wakes the CPU, rescheduling the CPU if it's not already active. */
    void wakeCPU();

    /** Gets a free thread id. Use if thread ids change across system. */
    int getFreeTid();

    // LL/SC debug functionality
    unsigned stCondFails;
    unsigned readStCondFailures() { return stCondFails; }
    unsigned setStCondFailures(unsigned st_fails) { return stCondFails = st_fails; }

    /** Returns a pointer to a thread context. */
    ThreadContext *tcBase(unsigned tid = 0)
    {
        return thread[tid]->getTC();
    }

    /** Count the Total Instructions Committed in the CPU. */
    virtual Counter totalInstructions() const
    {
        Counter total(0);

        for (int i=0; i < thread.size(); i++)
            total += thread[i]->numInst;

        return total;
    }

    /** The global sequence number counter. */
    InstSeqNum globalSeqNum[ThePipeline::MaxThreads];

    /** The global event number counter. */
    InstSeqNum cpuEventNum;

    /** Counter of how many stages have completed switching out. */
    int switchCount;

    /** Pointers to all of the threads in the CPU. */
    std::vector<Thread *> thread;

    /** Pointer to the icache interface. */
    MemInterface *icacheInterface;

    /** Pointer to the dcache interface. */
    MemInterface *dcacheInterface;

    /** Whether or not the CPU should defer its registration. */
    bool deferRegistration;

    /** Per-Stage Instruction Tracing */
    bool stageTracing;

    /** Is there a context switch pending? */
    bool contextSwitch;

    /** Threads Scheduled to Enter CPU */
    std::list<int> cpuWaitList;

    /** The cycle that the CPU was last running, used for statistics. */
    Tick lastRunningCycle;

    /** Number of Threads the CPU can process */
    unsigned numThreads;

    /** Number of Virtual Processors the CPU can process */
    unsigned numVirtProcs;

    /** Update Thread , used for statistic purposes*/
    inline void tickThreadStats();

    /** Per-Thread Tick */
    Stats::Vector threadCycles;

    /** Tick for SMT */
    Stats::Scalar smtCycles;

    /** Stat for total number of times the CPU is descheduled. */
    Stats::Scalar timesIdled;

    /** Stat for total number of cycles the CPU spends descheduled. */
    Stats::Scalar idleCycles;

    /** Stat for the number of committed instructions per thread. */
    Stats::Vector committedInsts;

    /** Stat for the number of committed instructions per thread. */
    Stats::Vector smtCommittedInsts;

    /** Stat for the total number of committed instructions. */
    Stats::Scalar totalCommittedInsts;

    /** Stat for the CPI per thread. */
    Stats::Formula cpi;

    /** Stat for the SMT-CPI per thread. */
    Stats::Formula smtCpi;

    /** Stat for the total CPI. */
    Stats::Formula totalCpi;

    /** Stat for the IPC per thread. */
    Stats::Formula ipc;

    /** Stat for the total IPC. */
    Stats::Formula smtIpc;

    /** Stat for the total IPC. */
    Stats::Formula totalIpc;
};

#endif // __CPU_O3_CPU_HH__
