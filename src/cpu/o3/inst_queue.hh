/*
 * Copyright (c) 2011-2012, 2014 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved.
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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

#ifndef __CPU_O3_INST_QUEUE_HH__
#define __CPU_O3_INST_QUEUE_HH__

#include <list>
#include <map>
#include <queue>
#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/dep_graph.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/o3/limits.hh"
#include "cpu/o3/mem_dep_unit.hh"
#include "cpu/o3/store_set.hh"
#include "cpu/op_class.hh"
#include "cpu/timebuf.hh"
#include "enums/SMTQueuePolicy.hh"
#include "sim/eventq.hh"

namespace gem5
{

struct BaseO3CPUParams;

namespace memory
{
class MemInterface;
} // namespace memory

namespace o3
{

class FUPool;
class CPU;
class IEW;

/**
 * A standard instruction queue class.  It holds ready instructions, in
 * order, in seperate priority queues to facilitate the scheduling of
 * instructions.  The IQ uses a separate linked list to track dependencies.
 * Similar to the rename map and the free list, it expects that
 * floating point registers have their indices start after the integer
 * registers (ie with 96 int and 96 fp registers, regs 0-95 are integer
 * and 96-191 are fp).  This remains true even for both logical and
 * physical register indices. The IQ depends on the memory dependence unit to
 * track when memory operations are ready in terms of ordering; register
 * dependencies are tracked normally. Right now the IQ also handles the
 * execution timing; this is mainly to allow back-to-back scheduling without
 * requiring IEW to be able to peek into the IQ. At the end of the execution
 * latency, the instruction is put into the queue to execute, where it will
 * have the execute() function called on it.
 * @todo: Make IQ able to handle multiple FU pools.
 */
class InstructionQueue
{
  public:
    // Typedef of iterator through the list of instructions.
    typedef typename std::list<DynInstPtr>::iterator ListIt;

    /** FU completion event class. */
    class FUCompletion : public Event
    {
      private:
        /** Executing instruction. */
        DynInstPtr inst;

        /** Index of the FU used for executing. */
        int fuIdx;

        /** Pointer back to the instruction queue. */
        InstructionQueue *iqPtr;

        /** Should the FU be added to the list to be freed upon
         * completing this event.
         */
        bool freeFU;

      public:
        /** Construct a FU completion event. */
        FUCompletion(const DynInstPtr &_inst, int fu_idx,
                     InstructionQueue *iq_ptr);

        virtual void process();
        virtual const char *description() const;

        void
        setFreeFU()
        {
            freeFU = true;
        }
    };

    /** Constructs an IQ. */
    InstructionQueue(CPU *cpu_ptr, IEW *iew_ptr,
                     const BaseO3CPUParams &params);

    /** Destructs the IQ. */
    ~InstructionQueue();

    /** Returns the name of the IQ. */
    std::string name() const;

    /** Resets all instruction queue state. */
    void resetState();

    /** Sets active threads list. */
    void setActiveThreads(std::list<ThreadID> *at_ptr);

    /** Sets the timer buffer between issue and execute. */
    void setIssueToExecuteQueue(TimeBuffer<IssueStruct> *i2eQueue);

    /** Sets the global time buffer. */
    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    /** Determine if we are drained. */
    bool isDrained() const;

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /** Takes over execution from another CPU's thread. */
    void takeOverFrom();

    /** Number of entries needed for given amount of threads. */
    int entryAmount(ThreadID num_threads);

    /** Resets max entries for all threads. */
    void resetEntries();

    /** Returns total number of free entries. */
    unsigned numFreeEntries();

    /** Returns number of free entries for a thread. */
    unsigned numFreeEntries(ThreadID tid);

    /** Returns whether or not the IQ is full. */
    bool isFull();

    /** Returns whether or not the IQ is full for a specific thread. */
    bool isFull(ThreadID tid);

    /** Returns if there are any ready instructions in the IQ. */
    bool hasReadyInsts();

    /** Inserts a new instruction into the IQ. */
    void insert(const DynInstPtr &new_inst);

    /** Inserts a new, non-speculative instruction into the IQ. */
    void insertNonSpec(const DynInstPtr &new_inst);

    /** Inserts a memory or write barrier into the IQ to make sure
     *  loads and stores are ordered properly.
     */
    void insertBarrier(const DynInstPtr &barr_inst);

    /** Returns the oldest scheduled instruction, and removes it from
     * the list of instructions waiting to execute.
     */
    DynInstPtr getInstToExecute();

    /** Gets a memory instruction that was referred due to a delayed DTB
     *  translation if it is now ready to execute.  NULL if none available.
     */
    DynInstPtr getDeferredMemInstToExecute();

    /** Gets a memory instruction that was blocked on the cache. NULL if none
     *  available.
     */
    DynInstPtr getBlockedMemInstToExecute();

    /**
     * Records the instruction as the producer of a register without
     * adding it to the rest of the IQ.
     */
    void
    recordProducer(const DynInstPtr &inst)
    {
        addToProducers(inst);
    }

    /** Process FU completion event. */
    void processFUCompletion(const DynInstPtr &inst, int fu_idx);

    /**
     * Schedules ready instructions, adding the ready ones (oldest first) to
     * the queue to execute.
     */
    void scheduleReadyInsts();

    /** Schedules a single specific non-speculative instruction. */
    void scheduleNonSpec(const InstSeqNum &inst);

    /**
     * Commits all instructions up to and including the given sequence number,
     * for a specific thread.
     */
    void commit(const InstSeqNum &inst, ThreadID tid = 0);

    /** Wakes all dependents of a completed instruction. */
    int wakeDependents(const DynInstPtr &completed_inst);

    /** Adds a ready memory instruction to the ready list. */
    void addReadyMemInst(const DynInstPtr &ready_inst);

    /**
     * Reschedules a memory instruction. It will be ready to issue once
     * replayMemInst() is called.
     */
    void rescheduleMemInst(const DynInstPtr &resched_inst);

    /** Replays a memory instruction. It must be rescheduled first. */
    void replayMemInst(const DynInstPtr &replay_inst);

    /**
     * Defers a memory instruction when its DTB translation incurs a hw
     * page table walk.
     */
    void deferMemInst(const DynInstPtr &deferred_inst);

    /**  Defers a memory instruction when it is cache blocked. */
    void blockMemInst(const DynInstPtr &blocked_inst);

    /**  Notify instruction queue that a previous blockage has resolved */
    void cacheUnblocked();

    /** Indicates an ordering violation between a store and a load. */
    void violation(const DynInstPtr &store, const DynInstPtr &faulting_load);

    /**
     * Squashes instructions for a thread. Squashing information is obtained
     * from the time buffer.
     */
    void squash(ThreadID tid);

    /** Returns the number of used entries for a thread. */
    unsigned
    getCount(ThreadID tid)
    {
        return count[tid];
    };

    /** Debug function to print all instructions. */
    void printInsts();

  private:
    /** Does the actual squashing. */
    void doSquash(ThreadID tid);

    /////////////////////////
    // Various pointers
    /////////////////////////

    /** Pointer to the CPU. */
    CPU *cpu;

    /** Cache interface. */
    memory::MemInterface *dcacheInterface;

    /** Pointer to IEW stage. */
    IEW *iewStage;

    /** The memory dependence unit, which tracks/predicts memory dependences
     *  between instructions.
     */
    MemDepUnit memDepUnit[MaxThreads];

    /** The queue to the execute stage.  Issued instructions will be written
     *  into it.
     */
    TimeBuffer<IssueStruct> *issueToExecuteQueue;

    /** The backwards time buffer. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to read information from timebuffer. */
    typename TimeBuffer<TimeStruct>::wire fromCommit;

    /** Function unit pool. */
    FUPool *fuPool;

    //////////////////////////////////////
    // Instruction lists, ready queues, and ordering
    //////////////////////////////////////

    /** List of all the instructions in the IQ (some of which may be issued).
     */
    std::list<DynInstPtr> instList[MaxThreads];

    /** List of instructions that are ready to be executed. */
    std::list<DynInstPtr> instsToExecute;

    /** List of instructions waiting for their DTB translation to
     *  complete (hw page table walk in progress).
     */
    std::list<DynInstPtr> deferredMemInsts;

    /** List of instructions that have been cache blocked. */
    std::list<DynInstPtr> blockedMemInsts;

    /** List of instructions that were cache blocked, but a retry has been seen
     * since, so they can now be retried. May fail again go on the blocked
     * list.
     */
    std::list<DynInstPtr> retryMemInsts;

    /**
     * Struct for comparing entries to be added to the priority queue.
     * This gives reverse ordering to the instructions in terms of
     * sequence numbers: the instructions with smaller sequence
     * numbers (and hence are older) will be at the top of the
     * priority queue.
     */
    struct PqCompare
    {
        bool operator()(const DynInstPtr &lhs, const DynInstPtr &rhs) const;
    };

    typedef std::priority_queue<DynInstPtr, std::vector<DynInstPtr>, PqCompare>
        ReadyInstQueue;

    /** List of ready instructions, per op class.  They are separated by op
     *  class to allow for easy mapping to FUs.
     */
    ReadyInstQueue readyInsts[Num_OpClasses];

    /** List of non-speculative instructions that will be scheduled
     *  once the IQ gets a signal from commit.  While it's redundant to
     *  have the key be a part of the value (the sequence number is stored
     *  inside of DynInst), when these instructions are woken up only
     *  the sequence number will be available.  Thus it is most efficient to be
     *  able to search by the sequence number alone.
     */
    std::map<InstSeqNum, DynInstPtr> nonSpecInsts;

    typedef std::map<InstSeqNum, DynInstPtr>::iterator NonSpecMapIt;

    /** Entry for the list age ordering by op class. */
    struct ListOrderEntry
    {
        OpClass queueType;
        InstSeqNum oldestInst;
    };

    /** List that contains the age order of the oldest instruction of each
     *  ready queue.  Used to select the oldest instruction available
     *  among op classes.
     *  @todo: Might be better to just move these entries around instead
     *  of creating new ones every time the position changes due to an
     *  instruction issuing.  Not sure std::list supports this.
     */
    std::list<ListOrderEntry> listOrder;

    typedef typename std::list<ListOrderEntry>::iterator ListOrderIt;

    /** Tracks if each ready queue is on the age order list. */
    bool queueOnList[Num_OpClasses];

    /** Iterators of each ready queue.  Points to their spot in the age order
     *  list.
     */
    ListOrderIt readyIt[Num_OpClasses];

    /** Add an op class to the age order list. */
    void addToOrderList(OpClass op_class);

    /**
     * Called when the oldest instruction has been removed from a ready queue;
     * this places that ready queue into the proper spot in the age order list.
     */
    void moveToYoungerInst(ListOrderIt age_order_it);

    DependencyGraph<DynInstPtr> dependGraph;

    //////////////////////////////////////
    // Various parameters
    //////////////////////////////////////

    /** IQ sharing policy for SMT. */
    SMTQueuePolicy iqPolicy;

    /** Number of Total Threads*/
    ThreadID numThreads;

    /** Pointer to list of active threads. */
    std::list<ThreadID> *activeThreads;

    /** Per Thread IQ count */
    unsigned count[MaxThreads];

    /** Max IQ Entries Per Thread */
    unsigned maxEntries[MaxThreads];

    /** Number of free IQ entries left. */
    unsigned freeEntries;

    /** The number of entries in the instruction queue. */
    unsigned numEntries;

    /** The total number of instructions that can be issued in one cycle. */
    unsigned totalWidth;

    /** The number of physical registers in the CPU. */
    unsigned numPhysRegs;

    /** Number of instructions currently in flight to FUs */
    int wbOutstanding;

    /** Delay between commit stage and the IQ.
     *  @todo: Make there be a distinction between the delays within IEW.
     */
    Cycles commitToIEWDelay;

    /** The sequence number of the squashed instruction. */
    InstSeqNum squashedSeqNum[MaxThreads];

    /** A cache of the recently woken registers.  It is 1 if the register
     *  has been woken up recently, and 0 if the register has been added
     *  to the dependency graph and has not yet received its value.  It
     *  is basically a secondary scoreboard, and should pretty much mirror
     *  the scoreboard that exists in the rename map.
     */
    std::vector<bool> regScoreboard;

    /** Adds an instruction to the dependency graph, as a consumer. */
    bool addToDependents(const DynInstPtr &new_inst);

    /** Adds an instruction to the dependency graph, as a producer. */
    void addToProducers(const DynInstPtr &new_inst);

    /** Moves an instruction to the ready queue if it is ready. */
    void addIfReady(const DynInstPtr &inst);

    /** Debugging function to count how many entries are in the IQ.  It does
     *  a linear walk through the instructions, so do not call this function
     *  during normal execution.
     */
    int countInsts();

    /** Debugging function to dump all the list sizes, as well as print
     *  out the list of nonspeculative instructions.  Should not be used
     *  in any other capacity, but it has no harmful sideaffects.
     */
    void dumpLists();

    /** Debugging function to dump out all instructions that are in the
     *  IQ.
     */
    void dumpInsts();

    struct IQStats : public statistics::Group
    {
        IQStats(CPU *cpu, const unsigned &total_width);
        /** Stat for number of instructions added. */
        statistics::Scalar instsAdded;
        /** Stat for number of non-speculative instructions added. */
        statistics::Scalar nonSpecInstsAdded;

        statistics::Scalar instsIssued;
        /** Stat for number of integer instructions issued. */
        statistics::Scalar intInstsIssued;
        /** Stat for number of floating point instructions issued. */
        statistics::Scalar floatInstsIssued;
        /** Stat for number of branch instructions issued. */
        statistics::Scalar branchInstsIssued;
        /** Stat for number of memory instructions issued. */
        statistics::Scalar memInstsIssued;
        /** Stat for number of miscellaneous instructions issued. */
        statistics::Scalar miscInstsIssued;
        /** Stat for number of squashed instructions that were ready to
         *  issue. */
        statistics::Scalar squashedInstsIssued;
        /** Stat for number of squashed instructions examined when
         *  squashing. */
        statistics::Scalar squashedInstsExamined;
        /** Stat for number of squashed instruction operands examined when
         * squashing.
         */
        statistics::Scalar squashedOperandsExamined;
        /** Stat for number of non-speculative instructions removed due to
         *  a squash.
         */
        statistics::Scalar squashedNonSpecRemoved;
        // Also include number of instructions rescheduled and replayed.

        /** Distribution of number of instructions in the queue.
         * @todo: Need to create struct to track the entry time for each
         * instruction. */
        // statistics::VectorDistribution queueResDist;
        /** Distribution of the number of instructions issued. */
        statistics::Distribution numIssuedDist;
        /** Distribution of the cycles it takes to issue an instruction.
         * @todo: Need to create struct to track the ready time for each
         * instruction. */
        // statistics::VectorDistribution issueDelayDist;

        /** Number of times an instruction could not be issued because a
         * FU was busy.
         */
        statistics::Vector statFuBusy;
        // statistics::Vector dist_unissued;
        /** Stat for total number issued for each instruction type. */
        statistics::Vector2d statIssuedInstType;

        /** Number of instructions issued per cycle. */
        statistics::Formula issueRate;

        /** Number of times the FU was busy. */
        statistics::Vector fuBusy;
        /** Number of times the FU was busy per instruction issued. */
        statistics::Formula fuBusyRate;
    } iqStats;

  public:
    struct IQIOStats : public statistics::Group
    {
        IQIOStats(statistics::Group *parent);
        statistics::Scalar intInstQueueReads;
        statistics::Scalar intInstQueueWrites;
        statistics::Scalar intInstQueueWakeupAccesses;
        statistics::Scalar fpInstQueueReads;
        statistics::Scalar fpInstQueueWrites;
        statistics::Scalar fpInstQueueWakeupAccesses;
        statistics::Scalar vecInstQueueReads;
        statistics::Scalar vecInstQueueWrites;
        statistics::Scalar vecInstQueueWakeupAccesses;

        statistics::Scalar intAluAccesses;
        statistics::Scalar fpAluAccesses;
        statistics::Scalar vecAluAccesses;
    } iqIOStats;
};

} // namespace o3
} // namespace gem5

#endif //__CPU_O3_INST_QUEUE_HH__
